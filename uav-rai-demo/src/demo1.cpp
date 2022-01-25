// System
#include <chrono>
#include <thread>
#include <mutex>

// RAI
#include "experiment.h"
#include "ex_droneRace.h"

#ifndef MARC_BUILD
// ROS
#include <rclcpp/rclcpp.hpp>
#include <tf2_eigen/tf2_eigen.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <rcl_interfaces/srv/set_parameters.hpp>

#include <motion_capture_tracking_interfaces/msg/named_pose_array.hpp>
#include <crazyswarm2_interfaces/msg/full_state.hpp>
#include <crazyswarm2_interfaces/srv/takeoff.hpp>
#include <crazyswarm2_interfaces/srv/land.hpp>
#include <crazyswarm2_interfaces/srv/notify_setpoints_stop.hpp>

using std::placeholders::_1;
using std::placeholders::_2;

using rcl_interfaces::srv::SetParameters;

using crazyswarm2_interfaces::msg::FullState;
using crazyswarm2_interfaces::srv::Land;
using crazyswarm2_interfaces::srv::Takeoff;
using crazyswarm2_interfaces::srv::NotifySetpointsStop;
using motion_capture_tracking_interfaces::msg::NamedPoseArray;

//===========================================================================


//===========================================================================

class DemoNode : public rclcpp::Node
{
public:
  DemoNode(bool display_only)
      : rclcpp::Node("demo1")
      , display_only_(display_only)
  {
    std::cout << "Starting demo1 node" << std::endl;
    sub_poses_=this->create_subscription<NamedPoseArray>(
        "poses", 1, std::bind(&DemoNode::posesChanged, this, _1));

    pub_full_state_ = this->create_publisher<FullState>("cf3/cmd_full_state", 10);

    client_takeoff_ = this->create_client<Takeoff>("cf3/takeoff");
    client_takeoff_->wait_for_service();

    client_land_ = this->create_client<Land>("cf3/land");
    client_land_->wait_for_service();

    client_notify_setpoints_stop_ = this->create_client<NotifySetpointsStop>("cf3/notify_setpoints_stop");
    client_notify_setpoints_stop_->wait_for_service();

    client_set_parameters_ = this->create_client<SetParameters>("crazyswarm2_server/set_parameters");
    client_set_parameters_->wait_for_service();

    q_real_ = zeros(3);
    qDot_real_ = zeros(3);
    done_ = false;

    this->declare_parameter<int>("control_frequency", 100);
    int f = this->get_parameter("control_frequency").as_int();

    timer_ = this->create_wall_timer(std::chrono::milliseconds(1000 / f), std::bind(&DemoNode::control_loop, this));

    if (!display_only_) {
      // Takeoff!
      auto request = std::make_shared<Takeoff::Request>();
      request->group_mask = 0;
      request->height = 0.5;
      request->duration = rclcpp::Duration::from_seconds(2);
      client_takeoff_->async_send_request(request);
      rclcpp::sleep_for(std::chrono::seconds(3));

      auto request2 = std::make_shared<SetParameters::Request>();
      request2->parameters.resize(1);
      request2->parameters[0].name = "cf3/params/usd/logging";
      request2->parameters[0].value.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
      request2->parameters[0].value.integer_value = 1;
      client_set_parameters_->async_send_request(request2);
    }

    // create a thread that runs the optimization in a loop
    // This replaces in my understanding the SequenceControllerExperiment class
    rai_thread_ = std::thread(&DemoNode::rai_thread, this);
  }

private:
  void rai_thread()
  {
    ex_droneRace(ex);
    done_ = true;

  }

  void posesChanged(const NamedPoseArray::SharedPtr msg)
  {
    static arr q_last;
    static rclcpp::Time time_last;

    // extract uav position and gate pose
    for (const auto& pose : msg->poses) {
      if (pose.name.find("target") == 0 && ex && !done_) {
        const std::lock_guard<std::mutex> lock(ex->mutex_C_);

        rai::Frame *f = ex->C.getFrame(pose.name.c_str(), false); //this needs a mutex!!
        const auto& pos = pose.pose.position;
        const auto& rot = pose.pose.orientation;
        auto Q = f->set_Q(); //that's not a mutex, by the way, more like a post-change-hook...
        Q->pos.set(pos.x, pos.y, pos.z);
        Q->rot.set(rot.w, rot.x, rot.y, rot.z);
      } else if (pose.name == "cf3") {
        const std::lock_guard<std::mutex> lock(mutex_mocap_);
        q_last = q_real_;
        q_real_ = {pose.pose.position.x, pose.pose.position.y, pose.pose.position.z};
        // numerically estimate qDot_real
        const double alpha = .1;
        const rclcpp::Time time(msg->header.stamp);
        if(time_last.nanoseconds() > 0){
          double dt = (time - time_last).seconds();
          qDot_real_ = (1.-alpha)*qDot_real_ + alpha*(q_real_ - q_last)/dt;
        }
        time_last = time;
      }
    }
  }

  void control_loop()
  {
    // static, so this will only be initialized in the first call
    static rclcpp::Time start_time = this->now();

    auto now = this->now();
    double ctrl_time = (now - start_time).seconds();

    // do nothing, if RAI isn't ready, yet
    if (!ex || !ex->bot) {
      return;
    }

    arr q_real, qDot_real;
    {//copy current state, while holding a lock (used multiple times)
      const std::lock_guard<std::mutex> lock(mutex_mocap_);
      q_real = q_real_;
      qDot_real = qDot_real_;
    }

    {//publish state
      auto stateSet = ex->bot->state.set(); //that's a mutex token
      stateSet->ctrlTime=ctrl_time;
      stateSet->q = q_real;
      stateSet->qDot = qDot_real;
    }

    
    // skip, if there is no valid spline
    if (ex->bot->getTimeToEnd()<=0.) {
      std::cout << "warning! time-to-end negative: " << ex->bot->getTimeToEnd() << std::endl;
      return;
    }

    // Land, if the KOMO thread was exited
    if (done_) {
      std::cout << "Landing!" << std::endl;

      if (!display_only_) {
        auto request0 = std::make_shared<SetParameters::Request>();
        request0->parameters.resize(1);
        request0->parameters[0].name = "cf3/params/usd/logging";
        request0->parameters[0].value.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
        request0->parameters[0].value.integer_value = 0;
        client_set_parameters_->async_send_request(request0);

        auto request1 = std::make_shared<NotifySetpointsStop::Request>();
        request1->remain_valid_millisecs = 100;
        request1->group_mask = 0;
        client_notify_setpoints_stop_->async_send_request(request1);

        auto request2 = std::make_shared<Land::Request>();
        request2->group_mask = 0;
        request2->height = 0.0;
        request2->duration = rclcpp::Duration::from_seconds(3.5);
        client_land_->async_send_request(request2);
      }

      timer_->cancel();
      return;
    }

    // everything looks good -> get state
    // sample spline at current time and compute pos, vel, acc
    arr pos, vel, acc;
    ex->bot->getReference(pos, vel, acc, q_real, qDot_real, ctrl_time); //this is mutex protected!

    FullState msg;
    msg.header.frame_id = "world";
    msg.header.stamp = this->now();
    msg.pose.position.x    = pos(0);
    msg.pose.position.y    = pos(1);
    msg.pose.position.z    = pos(2);
    msg.twist.linear.x     = vel(0);
    msg.twist.linear.y     = vel(1);
    msg.twist.linear.z     = vel(2);
    msg.acc.x              = acc(0);
    msg.acc.y              = acc(1);
    msg.acc.z              = acc(2);
    msg.pose.orientation.w = 1;
    msg.pose.orientation.x = 0;
    msg.pose.orientation.y = 0;
    msg.pose.orientation.z = 0;
    msg.twist.angular.x    = 0;
    msg.twist.angular.y    = 0;
    msg.twist.angular.z    = 0;
    if (!display_only_) {
      pub_full_state_->publish(msg);
    }
  }

  bool display_only_;

  std::unique_ptr<SecMPC_Experiments> ex;
  
  rclcpp::Publisher<FullState>::SharedPtr pub_full_state_;
  rclcpp::Subscription<NamedPoseArray>::SharedPtr sub_poses_;
  rclcpp::Client<Takeoff>::SharedPtr client_takeoff_;
  rclcpp::Client<Land>::SharedPtr client_land_;
  rclcpp::Client<NotifySetpointsStop>::SharedPtr client_notify_setpoints_stop_;
  rclcpp::Client<SetParameters>::SharedPtr client_set_parameters_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::thread rai_thread_;
  bool done_;

  std::mutex mutex_mocap_; // protects q_real_, qDot_real_
  arr q_real_;
  arr qDot_real_;
};

//===========================================================================

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rai::initCmdLine(argc, argv);

  // // Load our rai.cfg
  // std::string package_share_directory = ament_index_cpp::get_package_share_directory("uav-rai-demo");
  // std::string filename_cfg = package_share_directory + "/rai.cfg";
  // auto P = rai::getParameters();
  // std::ifstream file_cfg(filename_cfg);
  // if(file_cfg.good()) {
  //   file_cfg >> P();
  // }
  // file_cfg.close();


  //  rnd.clockSeed();
  rnd.seed(1);

  rclcpp::spin(std::make_shared<DemoNode>(argc > 1));
  rclcpp::shutdown();

  LOG(0) << " === bye bye ===\n used parameters:\n"
         << rai::getParameters()() << '\n';

  return 0;
}

#else

int main(int argc, char *argv[])
{
  rai::initCmdLine(argc, argv);
  rnd.seed(1);

  std::unique_ptr<SecMPC_Experiments> ex;

  ex_droneRace(ex);

  LOG(0) <<" === bye bye ===\n used parameters:\n" <<rai::getParameters()() <<'\n';

  return 0;
}

#endif
