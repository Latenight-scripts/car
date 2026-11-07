"""Compile actual car_ctr.cpp against in-memory ROS/UDP doubles. No sockets."""
import pathlib,tempfile,subprocess,json
ROOT=pathlib.Path(__file__).resolve().parents[1];OUT=pathlib.Path(__file__).parent
header=r'''
#pragma once
#include "common/udp_diagnostics.hpp"
#include <memory>
#include <functional>
#include <array>
#include <string>
#include <vector>
#include <sstream>
#include <cstring>
#include <cstdio>
#include <cstdlib>
#include <type_traits>
#include <unistd.h>
using namespace std;
inline vector<string> diag_messages,io_messages;inline double fake_now=0;inline int command_depth=0;
namespace control_msgs {namespace msg {
struct CarCtr {using SharedPtr=shared_ptr<CarCtr>;int source=1,ctr_mode=1,up_down_flag=0,enable=1,turnmode=0,heartbeat=1;float speed=0,angle=0;};
struct CarState {int enable=0,ctrmode=0,turnmode=0;array<float,4> speed{};array<int,8> errcode{};};}}
namespace std_msgs {namespace msg {struct Float32 {using SharedPtr=shared_ptr<Float32>;float data=0;};struct Float32MultiArray {};struct String {string data;};}}
inline function<void(control_msgs::msg::CarCtr::SharedPtr)> callback;
namespace rclcpp {
struct Logger {};
template<class T>struct Subscription{using SharedPtr=shared_ptr<Subscription<T>>;};
template<class T>struct Publisher{using SharedPtr=shared_ptr<Publisher<T>>;string topic;Publisher(string name=""):topic(name){}void publish(const T& m) {if constexpr(is_same_v<T,std_msgs::msg::String>) {if(topic=="/car_driver/diag")diag_messages.push_back(m.data);else io_messages.push_back(m.data);}}};
struct Node {using SharedPtr=shared_ptr<Node>;static SharedPtr make_shared(string){return std::make_shared<Node>();}
 template<class T>void declare_parameter(string,T){}
 template<class T>void get_parameter(string key,T& value){if constexpr(is_arithmetic_v<T>) {if(key=="run_enable")value=true;else if(key=="mode_change_time")value=3;}}
 template<class T>typename Publisher<T>::SharedPtr create_publisher(string name,int){return std::make_shared<Publisher<T>>(name);}
 template<class T,class F>typename Subscription<T>::SharedPtr create_subscription(string,int depth,F f){command_depth=depth;if constexpr(is_same_v<T,control_msgs::msg::CarCtr>)callback=f;return std::make_shared<Subscription<T>>();}
 Logger get_logger(){return {};}};
inline void init(int,char**){}inline bool ok(){return false;}inline void shutdown(){}inline void spin_some(Node::SharedPtr){}
struct Rate{Rate(int){}void sleep(){}};
}
#define RCLCPP_WARN(...) ((void)0)
#define RCLCPP_INFO(...) ((void)0)
struct TTimer {double begin=fake_now;void Clear(){begin=fake_now;}double GetValue(){return fake_now-begin;}};
struct CheckCell {void SetLimit(int){}void Beat(){}};
struct TNodeCheck {TNodeCheck(rclcpp::Node::SharedPtr,string){}CheckCell cell;CheckCell* Find(string){return &cell;}};
struct TUDP;inline TUDP* last_udp=nullptr;
struct FakeTap {void enable(bool){}vector<UdpDiagnosticPacket> take(){return {};}int dropped(){return 0;}};
struct TUDP {FakeTap diagnostics;int rec_count=0;char rec_buf[1000]{};vector<string> packets;TUDP(int){last_udp=this;}void AddRemoteIP(string,int){}void Send(char* p){packets.push_back(p);}};
inline vector<string> split(string a,string delim){vector<string> r;size_t at;while((at=a.find(delim))!=string::npos){r.push_back(a.substr(0,at));a.erase(0,at+delim.size());}r.push_back(a);return r;}
'''
test=r'''
#define main production_main
#include "SOURCE"
#undef main
#include <cassert>
int main(){
 fake_now=0;TCarCtr car;assert(command_depth==1);
 strcpy(last_udp->rec_buf,"Car 255 1 0");last_udp->rec_count=1;car.run();
 auto cmd=make_shared<control_msgs::msg::CarCtr>();cmd->turnmode=4;cmd->speed=0;callback(cmd);
 assert(last_udp->packets[last_udp->packets.size()-2]=="Car Run 0.0 -0.00");
 assert(last_udp->packets.back()=="Car SteerModeCtrl 4");
 fake_now=.1;strcpy(last_udp->rec_buf,"Car 255 1 4");last_udp->rec_count=1;car.run();callback(cmd);
 auto before=last_udp->packets.size();fake_now=3.05;cmd->speed=1.;callback(cmd);assert(last_udp->packets.size()==before);
 fake_now=3.2;callback(cmd);assert(last_udp->packets.back()=="Car Run -2.00 0.0");
 fake_now=3.8;car.run();assert(last_udp->packets.back()=="Car Run 0.0 -0.00");
 before=last_udp->packets.size();fake_now=3.85;cmd->turnmode=0;cmd->speed=0;cmd->angle=10;callback(cmd);
 assert(last_udp->packets[before]=="Car Run 0.0 -10.00");
 assert(last_udp->packets.back()=="Car SteerModeCtrl 0");
 fake_now=3.9;strcpy(last_udp->rec_buf,"Car 255 1 0;Wheel0 bad");last_udp->rec_count=1;car.run();
 puts(diag_messages.back().c_str());
 fake_now=4.0;strcpy(last_udp->rec_buf,"Wheel0 ignored 60");last_udp->rec_count=1;car.run();puts(diag_messages.back().c_str());
 for(const auto& io:io_messages)puts(io.c_str());
 // Disabled commands with a nonzero payload must issue exactly one stop,
 // without sending either a motion command or a new steering-mode request.
 fake_now=4.1;cmd->enable=1;cmd->turnmode=0;cmd->speed=0;callback(cmd);
 fake_now=7.2;cmd->enable=1;cmd->turnmode=0;cmd->speed=.5;callback(cmd);
 assert(last_udp->packets.back()=="Car Run 0.50 -10.00");
 before=last_udp->packets.size();cmd->enable=0;cmd->speed=.8;callback(cmd);
 assert(last_udp->packets.size()==before+1);
 assert(last_udp->packets.back()=="Car Run 0.0 -10.00");
 before=last_udp->packets.size();cmd->turnmode=4;callback(cmd);
 assert(last_udp->packets.size()==before+1);
 assert(last_udp->packets.back()=="Car Run 0.0 -10.00");
 cmd->enable=1;cmd->speed=0;callback(cmd);
 assert(last_udp->packets.back()=="Car SteerModeCtrl 4");
 before=last_udp->packets.size();cmd->enable=0;cmd->speed=.8;callback(cmd);
 assert(last_udp->packets.size()==before+1);
 assert(last_udp->packets.back()=="Car Run 0.0 -10.00");
 puts("C++17 protocol, mode delay, emergency stop and command watchdog passed");
}
'''
with tempfile.TemporaryDirectory(prefix='rda-cpp-') as folder:
 tmp=pathlib.Path(folder)
 includes=['common/public.hpp','common/myudp.hpp','rclcpp/rclcpp.hpp','geometry_msgs/msg/point.hpp','geometry_msgs/msg/twist.hpp','std_msgs/msg/float32.hpp','std_msgs/msg/string.hpp','std_msgs/msg/float32_multi_array.hpp','control_msgs/msg/car_state.hpp','control_msgs/msg/car_ctr.hpp']
 p=tmp/'common/udp_diagnostics.hpp';p.parent.mkdir(parents=True,exist_ok=True);p.write_text((ROOT/'msg/common/include/common/udp_diagnostics.hpp').read_text())
 (tmp/'double.hpp').write_text(header)
 for name in includes:
  p=tmp/name;p.parent.mkdir(parents=True,exist_ok=True);p.write_text('#include "double.hpp"\n')
 (tmp/'test.cpp').write_text(test.replace('SOURCE',str(ROOT/'control/car_ctr/src/car_ctr.cpp')))
 subprocess.run(['clang++','-std=c++17','-I',folder,str(tmp/'test.cpp'),'-o',str(tmp/'test')],check=True,capture_output=True,text=True)
 result=subprocess.run([str(tmp/'test')],check=True,capture_output=True,text=True)
 rows=[json.loads(line) for line in result.stdout.splitlines() if line.startswith('{')];diags=[r for r in rows if 'wheel_feedback_age_s' in r];assert diags[0]['wheel_feedback_age_s']==[None]*4;assert diags[1]['wheel_feedback_age_s'][0]==0 and diags[1]['wheel_feedback_age_s'][1:] == [None]*3;assert any(r.get('direction')=='rx_observed' for r in rows);print(result.stdout.splitlines()[-1]);(OUT/'cpp_protocol_result.json').write_text(json.dumps({'passed':True,'scope':'Actual C++ source with in-memory ROS/UDP doubles; not colcon/ROS ABI or firmware validation','output':result.stdout},indent=2))
