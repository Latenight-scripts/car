"""Exercise production ROS timer/callback order with in-memory ROS messages."""
import importlib
import json
from pathlib import Path
import sys
import time
import types

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / 'rda_dwa'), str(ROOT / 'rda_dwa_real'),
               str(ROOT / 'tests')]


@pytest.fixture
def bridge(monkeypatch):
    import ros_doubles as fake
    monkeypatch.setitem(fake.CONFIG['rda_dwa_real_bridge']['ros__parameters'],
                        'async_planning', False)
    original_init = fake.Message.__init__
    def init_message(self, **kwargs):
        original_init(self, **kwargs)
        # geometry_msgs/Twist exposes vectors directly; Odometry nests Twist.
        self.linear, self.angular = self.twist.linear, self.twist.angular
    monkeypatch.setattr(fake.Message, '__init__', init_message)
    fake.install_doubles()
    return importlib.import_module('rda_dwa_real.bridge_node').RealVehicleBridgeNode(), fake


def read_diag(node):
    def reject_nonfinite(value):
        raise AssertionError('Nonstandard JSON value: ' + value)
    return json.loads(node._pub_diag.messages[-1].data,
                      parse_constant=reject_nonfinite)


def localize(node, fake):
    msg = fake.Message()
    msg.pose = types.SimpleNamespace(pose=msg.pose)
    node._cb_odom(msg)


def test_timer_waits_for_goal_before_and_after_localization(bridge):
    node, fake = bridge
    for localized in (False, True):
        if localized:
            localize(node, fake)
        for _ in range(5):
            node._control_cycle()
            diag = read_diag(node)
            assert diag['reason'] == 'no_goal'
            assert diag['goal'] is None and diag['goal_received'] is False
            assert diag['spin_gap'] is None
            assert node._pub_cmd.messages[-1].linear.x == 0.
            assert json.loads(node._pub_plan.messages[-1].data)['reference'] == []


@pytest.mark.parametrize('goal', [(0., 0.), (5., 0.), (0., 2.5)])
def test_late_goal_flows_through_actual_control_cycle(bridge, goal):
    node, fake = bridge
    node._control_cycle()
    localize(node, fake)
    node._cb_obstacles(fake.Message())
    msg = fake.Message()
    msg.pose.position.x, msg.pose.position.y = goal
    node._cb_goal(msg)
    node._control_cycle()
    diag = read_diag(node)
    assert diag['goal_received'] is True and diag['goal'] == list(goal)
    packet = json.loads(node._pub_plan.messages[-1].data)
    if goal == (0., 0.):
        assert diag['reason'] == 'reached' and packet['reference'] == []
    elif goal == (5., 0.):
        assert diag['reason'] == 'motion' and len(packet['reference']) > 2
        assert node._pub_cmd.messages[-1].linear.x > 0.
    else:
        assert diag['reason'] == 'spin_requested' and packet['mode'] == 1
    session = node.core._session
    node._cb_goal(msg)
    assert node.core._session == session


def test_goal_waits_safely_for_missing_and_expired_inputs(bridge):
    node, fake = bridge
    msg = fake.Message()
    msg.pose.position.x = 5.
    node._cb_goal(msg)
    node._control_cycle()
    assert read_diag(node)['reason'] == 'localization_fault'
    localize(node, fake)
    node._control_cycle()
    assert read_diag(node)['reason'] == 'obstacle_timeout'
    node._cb_obstacles(fake.Message())
    node._control_cycle()
    assert read_diag(node)['reason'] == 'motion'
    node.core._t_obs = time.monotonic() - node.core.cfg.obstacle_timeout - 1.
    node._control_cycle()
    assert read_diag(node)['reason'] == 'obstacle_timeout'
    assert node._pub_cmd.messages[-1].linear.x == 0.


def test_spawned_planner_survives_startup_and_late_goal(bridge):
    node, fake = bridge
    from rda_dwa_real.async_planner import AsyncPlanner
    worker = AsyncPlanner(node.planner)
    node._async_planner = node.planner = node.core.planner = worker
    try:
        node._control_cycle()
        assert read_diag(node)['reason'] == 'no_goal'
        goal = fake.Message()
        goal.pose.position.x = 5.
        node._cb_goal(goal)
        deadline = time.monotonic() + 4.
        index = 0
        while time.monotonic() < deadline:
            index += 1
            loc = fake.Message()
            loc.pose = types.SimpleNamespace(pose=loc.pose)
            loc.header.stamp.nanosec = index * 1000000
            obs = fake.Message()
            obs.header.stamp.nanosec = index * 1000000
            node._cb_odom(loc)
            node._cb_obstacles(obs)
            node._control_cycle()
            if read_diag(node)['reason'] == 'motion':
                assert node._pub_cmd.messages[-1].linear.x > 0.
                break
            assert node._pub_cmd.messages[-1].linear.x == 0.
            time.sleep(.05)
        else:
            raise AssertionError(read_diag(node))
    finally:
        worker.close()
    assert not worker._process.is_alive()
