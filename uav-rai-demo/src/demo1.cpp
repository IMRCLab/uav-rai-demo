// System
#include <chrono>
#include <thread>
#include <mutex>

// RAI
#include <BotOp/bot.h>
#include <BotOp/SequenceController.h>
#include <KOMO/manipTools.h>
#include <KOMO/pathTools.h>
#include <Kin/viewer.h>

#include <Kin/F_forces.h>

// ROS
#include <rclcpp/rclcpp.hpp>
#include <tf2_eigen/tf2_eigen.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <motion_capture_tracking_interfaces/msg/named_pose_array.hpp>
#include <crazyswarm2_interfaces/msg/full_state.hpp>
#include <crazyswarm2_interfaces/srv/takeoff.hpp>
#include <crazyswarm2_interfaces/srv/land.hpp>
#include <crazyswarm2_interfaces/srv/notify_setpoints_stop.hpp>

// #define DISPLAY_ONLY

using std::placeholders::_1;
using std::placeholders::_2;

using crazyswarm2_interfaces::msg::FullState;
using crazyswarm2_interfaces::srv::Land;
using crazyswarm2_interfaces::srv::Takeoff;
using crazyswarm2_interfaces::srv::NotifySetpointsStop;
using motion_capture_tracking_interfaces::msg::NamedPoseArray;

//===========================================================================

struct SequenceControllerExperiment
{
  rai::Configuration &C;
  unique_ptr<SequenceController> ctrl;
  unique_ptr<BotOp> bot;
  Metronome tic;
  uint stepCount = 0;

  std::mutex mutex_C_;

  KOMO *__komo=0;
  double timeCost=1e0, ctrlCost=1e0;

  SequenceControllerExperiment(rai::Configuration& _C, ObjectiveL& phi, double cycleTime=.1)
    : C(_C),
      tic(cycleTime)
      {
  }

  SequenceControllerExperiment(rai::Configuration& _C, KOMO& komo, double cycleTime=.1, double timeCost=1e1, double ctrlCost=1e-2)
    : C(_C),
      tic(cycleTime),
      __komo(&komo),
      timeCost(timeCost), ctrlCost(ctrlCost){
  }

  bool step(ObjectiveL& phi){
    stepCount++;

    //-- start a robot thread
    if(!bot){
      bot = make_unique<BotOp>(C, rai::checkParameter<bool>("real"));
      bot->home(C);
      bot->setControllerWriteData(1);
      if(bot->optitrack) bot->optitrack->pull(C);
      rai::wait(.2);
    }

    if(!ctrl){
      //needs to be done AFTER bot initialization (optitrack..)
      if(__komo){
        ctrl = make_unique<SequenceController>(C, *__komo, C.getJointState(), timeCost, ctrlCost);
      }else{
        ctrl = make_unique<SequenceController>(C, phi, C.getJointState(), timeCost, ctrlCost);
      }
    }

    //-- iterate
    tic.waitForTic();

    //-- get optitrack
    if(bot->optitrack) bot->optitrack->pull(C);

    //-- get current state (time,q,qDot)
    arr q,qDot, q_ref, qDot_ref;
    double ctrlTime = 0.;
    bot->getState(q, qDot, ctrlTime);
    bot->getReference(q_ref, qDot_ref, NoArr, q, qDot, ctrlTime);

    //-- iterate MPC
    {
      const std::lock_guard<std::mutex> lock(mutex_C_);
      ctrl->cycle(C, phi, q_ref, qDot_ref, q, qDot, ctrlTime);
      ctrl->report(C, phi);
    }

    //-- send spline update
    auto sp = ctrl->getSpline(bot->get_t());
    if(sp.pts.N) bot->move(sp.pts, sp.vels, sp.times, true);

    //-- update C
    {
      const std::lock_guard<std::mutex> lock(mutex_C_);
      bot->step(C, .0);
    }
    if(bot->keypressed=='q' || bot->keypressed==27) return false;

    return true;
  }
};

//===========================================================================

class DemoNode : public rclcpp::Node
{
public:
  DemoNode()
      : rclcpp::Node("demo1")
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

    q_real_ = zeros(3);
    qDot_real_ = zeros(3);
    done_ = false;

    this->declare_parameter<int>("control_frequency", 100);
    int f = this->get_parameter("control_frequency").as_int();

    timer_ = this->create_wall_timer(std::chrono::milliseconds(1000 / f), std::bind(&DemoNode::control_loop, this));

#ifndef DISPLAY_ONLY
    // Takeoff!
    auto request = std::make_shared<Takeoff::Request>();
    request->group_mask = 0;
    request->height = 0.5;
    request->duration = rclcpp::Duration::from_seconds(2);
    client_takeoff_->async_send_request(request);
    rclcpp::sleep_for(std::chrono::seconds(3));
#endif

    // create a thread that runs the optimization in a loop
    // This replaces in my understanding the SequenceControllerExperiment class
    rai_thread_ = std::thread(&DemoNode::rai_thread, this);
  }

private:
  void rai_thread()
  {
    rai::Configuration C;

    // std::string package_share_directory = ament_index_cpp::get_package_share_directory("uav-rai-demo");
    // std::string filename_g = package_share_directory + "/droneRace.g";
    C.addFile("droneRace.g");

    // {
    //   const std::lock_guard<std::mutex> lock(mutex_mocap_);
    //   C["drone"]->setPosition({position_uav_.x(), position_uav_.y(), position_uav_.z()});

    //   Eigen::Quaterniond q(pose_gate_.rotation());

    //   C["target3"]->setPose(rai::Transformation(
    //       rai::Vector(pose_gate_.translation().x(),
    //                   pose_gate_.translation().y(),
    //                   pose_gate_.translation().z()),
    //       rai::Quaternion(q.w(), q.x(), q.y(), q.z()
    //     )));
    // }

    //-- define constraints
    KOMO komo;
    komo.setModel(C, false);
    komo.setTiming(10., 1, 5., 1);
    komo.add_qControlObjective({}, 1, 1e-1);
    komo.addQuaternionNorms();
    komo.addObjective({1.}, FS_positionDiff, {"drone", "target0_before"}, OT_eq, {1e1});
    komo.addObjective({2.}, FS_positionDiff, {"drone", "target0_after"}, OT_eq, {1e1});
    komo.addObjective({3.}, FS_positionDiff, {"drone", "target1_before"}, OT_eq, {1e1});
    komo.addObjective({4.}, FS_positionDiff, {"drone", "target1_after"}, OT_eq, {1e1});
    komo.addObjective({5.}, FS_positionDiff, {"drone", "target2_before"}, OT_eq, {1e1});
    komo.addObjective({6.}, FS_positionDiff, {"drone", "target2_after"}, OT_eq, {1e1});
    komo.addObjective({7.}, FS_positionDiff, {"drone", "target3_before"}, OT_eq, {1e1});
    komo.addObjective({8.}, FS_positionDiff, {"drone", "target3_after"}, OT_eq, {1e1});
    komo.addObjective({9.}, FS_positionDiff, {"drone", "target0_before"}, OT_eq, {1e1});
    komo.addObjective({10.}, FS_positionDiff, {"drone", "target0_after"}, OT_eq, {1e1});

    ex = std::make_unique<SequenceControllerExperiment>(C, komo, .1, 1e0, 2);
    ex->step(komo.objectives);
    ex->ctrl->tauCutoff = .1;
    
    while(ex->step(komo.objectives)){
      if(ex->ctrl->timingMPC.phase==8){ //hard code endless loop by phase backtracking
        ex->ctrl->timingMPC.update_setPhase(0);
      }
    }

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
    std::cout << "t= " << ctrl_time << std::endl;

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
      return;
    }

    // Land, if the KOMO thread was exited
    if (done_) {
      std::cout << "Landing!" << std::endl;
#ifndef DISPLAY_ONLY
      auto request1 = std::make_shared<NotifySetpointsStop::Request>();
      request1->remain_valid_millisecs = 100;
      request1->group_mask = 0;
      client_notify_setpoints_stop_->async_send_request(request1);

      auto request2 = std::make_shared<Land::Request>();
      request2->group_mask = 0;
      request2->height = 0.0;
      request2->duration = rclcpp::Duration::from_seconds(3.5);
      client_land_->async_send_request(request2);
#endif

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
#ifndef DISPLAY_ONLY
    pub_full_state_->publish(msg);
#endif
  }

  std::unique_ptr<SequenceControllerExperiment> ex;
  
  rclcpp::Publisher<FullState>::SharedPtr pub_full_state_;
  rclcpp::Subscription<NamedPoseArray>::SharedPtr sub_poses_;
  rclcpp::Client<Takeoff>::SharedPtr client_takeoff_;
  rclcpp::Client<Land>::SharedPtr client_land_;
  rclcpp::Client<NotifySetpointsStop>::SharedPtr client_notify_setpoints_stop_;
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

  rclcpp::spin(std::make_shared<DemoNode>());
  rclcpp::shutdown();

  LOG(0) << " === bye bye ===\n used parameters:\n"
         << rai::getParameters()() << '\n';

  return 0;
}
