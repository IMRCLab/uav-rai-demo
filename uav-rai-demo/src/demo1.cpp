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
#include <motion_capture_tracking_interfaces/msg/named_pose_array.hpp>
#include "crazyswarm2_interfaces/msg/full_state.hpp"
#include "crazyswarm2_interfaces/srv/takeoff.hpp"
#include "crazyswarm2_interfaces/srv/land.hpp"
#include "crazyswarm2_interfaces/srv/notify_setpoints_stop.hpp"

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
  arr qHome;
  unique_ptr<SequenceController> ctrl;
  unique_ptr<BotOp> bot;
  Metronome tic;
  uint stepCount = 0;

  SequenceControllerExperiment(rai::Configuration &_C, ObjectiveL &phi, double cycleTime = .1) : C(_C), tic(cycleTime)
  {
    qHome = C.getJointState();
    ctrl = make_unique<SequenceController>(C, phi, qHome);
  }

  SequenceControllerExperiment(rai::Configuration &_C, const KOMO &komo, double cycleTime = .1) : C(_C), tic(cycleTime)
  {
    qHome = C.getJointState();
    ctrl = make_unique<SequenceController>(C, komo, qHome);
  }

  bool step(ObjectiveL &phi)
  {
    stepCount++;

    //-- start a robot thread
    if (!bot)
    {
      bot = make_unique<BotOp>(C, rai::checkParameter<bool>("real"));
      bot->home(C);
      bot->setControllerWriteData(1);
      rai::wait(.2);
    }

    //-- iterate
    tic.waitForTic();

    //-- get optitrack
    if (bot->optitrack)
      bot->optitrack->pull(C);

    //-- get current state (time,q,qDot)
    arr q, qDot, q_ref, qDot_ref;
    double ctrlTime = 0.;
    bot->getState(q, qDot, ctrlTime);
    bot->getReference(q_ref, qDot_ref, NoArr, q, qDot, ctrlTime);

    //-- iterate MPC
    ctrl->cycle(C, phi, q_ref, qDot_ref, q, qDot, ctrlTime);
    ctrl->report(C, phi);

    //-- send leap target
    auto sp = ctrl->getSpline(bot->get_t());
    if (sp.pts.N)
      bot->move(sp.pts, sp.vels, sp.times, true);

    //-- update C
    bot->step(C, .0);
    if (bot->keypressed == 'q' || bot->keypressed == 27)
      return false;

    return true;
  }
};

//===========================================================================

void testDroneRace()
{
  rai::Configuration C;
  C.addFile("droneRace.g");

  //-- define constraints
  KOMO komo;
  komo.setModel(C, false);
  komo.setTiming(7., 1, 5., 1);
  komo.add_qControlObjective({}, 1, 1e-1);
  komo.addQuaternionNorms();
  komo.addObjective({1.}, FS_positionDiff, {"drone", "target0"}, OT_eq, {1e1});
  komo.addObjective({2.}, FS_positionDiff, {"drone", "target1"}, OT_eq, {1e1});
  komo.addObjective({3.}, FS_positionDiff, {"drone", "target2"}, OT_eq, {1e1});
  komo.addObjective({4.}, FS_positionDiff, {"drone", "target3"}, OT_eq, {1e1});
  komo.addObjective({5.}, FS_positionDiff, {"drone", "target0"}, OT_eq, {1e1});
  komo.addObjective({6.}, FS_positionDiff, {"drone", "target1"}, OT_eq, {1e1});
  komo.addObjective({7.}, FS_position, {"drone"}, OT_eq, {1e1}, {0, -.5, 1.});

  //-- not yet integrated
  //SequenceControllerExperiment ex(C, komo);
  //while(ex.step(komo.objectives));

  //-- manually just optimize once and dump spline

  //optimize keyframes
  komo.optimize();
  komo.getReport(true);
  komo.view(false, "optimized motion");
  arr keyframes = komo.getPath_qOrg();

  //optimize timing
  TimingMPC F(keyframes, 1e0, 10); //last number (ctrlCost) in range [1,10] from fast-slow
  arr x0 = C["drone"]->getPosition();
  arr v0 = zeros(3);
  F.solve(x0, v0);

  //get spline
  rai::CubicSpline S;
  F.getCubicSpline(S, x0, v0);

  //analyze only to plot the max vel/acc
  arr path = S.eval(range(0., S.times.last(), 100));
  double tau = S.times.last() / 100.;
  arr ttau = consts<double>(tau, 101);
  double maxVel = 1., maxAcc = 1., maxJer = 30.;
  arr time(path.d0);
  time.setZero();
  for (uint t = 1; t < time.N; t++)
    time(t) = time(t - 1) + ttau(t);

  arr v = max(getVel(path, ttau), 1) / maxVel;
  arr a = max(getAcc(path, ttau), 1) / maxAcc;
  arr j = max(getJerk(path, ttau), 1) / maxJer;
  arr vi = min(getVel(path, ttau), 1) / maxVel;
  arr ai = min(getAcc(path, ttau), 1) / maxAcc;
  arr ji = min(getJerk(path, ttau), 1) / maxJer;
  catCol(LIST(~~time, ~~v, ~~a, ~~j, ~~vi, ~~ai, ~~ji)).reshape(-1, 7).writeRaw(FILE("z.dat"));
  gnuplot("plot [:][-1.1:1.1] 'z.dat' us 1:2 t 'vmax', '' us 1:3 t 'amax', '' us 1:5 t 'vmin', '' us 1:6 t 'amin'"); //, '' us 1:4 t 'j', , '' us 1:7 t 'jmin'

  //display
  rai::Mesh M;
  M.V = S.eval(range(0., S.times.last(), 100));
  M.makeLineStrip();
  C.gl()->add(M);
  C.watch(true);

  //just sample & dump the spline
  for (double t = 0; t < S.times.last(); t += .01)
  {
    //time 3-positions 3-velocities
    cout << t << S.eval(t).modRaw() << ' ' << S.eval(t, 1).modRaw() << endl;
  }
}

//===========================================================================

class DemoNode : public rclcpp::Node
{
public:
  DemoNode()
      : rclcpp::Node("demo1")
  {
    sub_poses_ = this->create_subscription<NamedPoseArray>(
        "poses", 1, std::bind(&DemoNode::posesChanged, this, _1));

    pub_full_state_ = this->create_publisher<FullState>("cf3/cmd_full_state", 10);

    client_takeoff_ = this->create_client<Takeoff>("cf3/takeoff");
    client_takeoff_->wait_for_service();

    client_land_ = this->create_client<Land>("cf3/land");
    client_land_->wait_for_service();

    client_notify_setpoints_stop_ = this->create_client<NotifySetpointsStop>("cf3/notify_setpoints_stop");
    client_notify_setpoints_stop_->wait_for_service();

    this->declare_parameter<int>("control_frequency", 100);
    int f = this->get_parameter("control_frequency").as_int();

#ifndef DISPLAY_ONLY
    timer_ = this->create_wall_timer(std::chrono::milliseconds(1000 / f), std::bind(&DemoNode::control_loop, this));

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
    C.addFile("droneRace.g");

    {
      const std::lock_guard<std::mutex> lock(mutex_mocap_);
      C["drone"]->setPosition({position_uav_.x(), position_uav_.y(), position_uav_.z()});

      Eigen::Quaterniond q(pose_gate_.rotation());

      C["target3"]->setPose(rai::Transformation(
          rai::Vector(pose_gate_.translation().x(),
                      pose_gate_.translation().y(),
                      pose_gate_.translation().z()),
          rai::Quaternion(q.w(), q.x(), q.y(), q.z()
        )));
    }

    //-- define constraints
    KOMO komo;
    komo.setModel(C, false);
    komo.setTiming(7., 1, 5., 1);
    komo.add_qControlObjective({}, 1, 1e-1);
    komo.addQuaternionNorms();
    komo.addObjective({1.}, FS_positionDiff, {"drone", "target0"}, OT_eq, {1e1});
    komo.addObjective({2.}, FS_positionDiff, {"drone", "target1"}, OT_eq, {1e1});
    komo.addObjective({3.}, FS_positionDiff, {"drone", "target2"}, OT_eq, {1e1});
    komo.addObjective({4.}, FS_positionDiff, {"drone", "target3"}, OT_eq, {1e1});
    komo.addObjective({5.}, FS_positionDiff, {"drone", "target0"}, OT_eq, {1e1});
    komo.addObjective({6.}, FS_positionDiff, {"drone", "target1"}, OT_eq, {1e1});
    komo.addObjective({7.}, FS_position, {"drone"}, OT_eq, {1e1}, {0, -.5, 1.});

    //-- not yet integrated
    ex = make_unique<SequenceControllerExperiment>(C, komo);
    while(ex.step(komo.objectives));

    ex = make_unique<SequenceControllerExperiment>(C, komo, .1, 1e0, 1e0);
    ex->step(komo.objectives);
    ex.ctrl->tauCutoff = .1;
    
    while(ex->step(komo.objectives)){
      if(ex->ctrl->timingMPC.phase==5){ //hard code endless loop by phase backtracking
	ex->ctrl->timingMPC.update_setPhase(1);
      }


      //MISSING: update the gates and UAV pose
    }

  }

  void posesChanged(const NamedPoseArray::SharedPtr msg)
  {
    // extract uav position and gate pose
    for (const auto& pose : msg->poses) {
      if (pose.name == "gate") {
        const std::lock_guard<std::mutex> lock(mutex_mocap_);
        tf2::fromMsg(pose.pose, pose_gate_);
      } else if (pose.name == "cf3") {
        const std::lock_guard<std::mutex> lock(mutex_mocap_);
        tf2::fromMsg(pose.pose.position, position_uav_);
      }
    }
  }

  void control_loop()
  {
    // sample spline at current time and compute pos, vel, acc
    arr pos, vel, acc;

    auto now = std::chrono::steady_clock::now();
    double t = std::chrono::duration_cast<std::chrono::milliseconds>(now - spline_start_).count() / 1000.0f;
    bool land = false;

    {
      if (ex->bot->getTimeToEnd()<=0.) {
        land = true;
      } else {
        ex->bot->getReference(pos, vel, acc, NoArr, NoArr, t); //this is mutex protected!
      }
    }

    if (land) {
      auto request1 = std::make_shared<NotifySetpointsStop::Request>();
      request1->remain_valid_millisecs = 100;
      request1->group_mask = 0;
      client_notify_setpoints_stop_->async_send_request(request1);

      auto request2 = std::make_shared<Land::Request>();
      request2->group_mask = 0;
      request2->height = 0.0;
      request2->duration = rclcpp::Duration::from_seconds(3.5);
      client_land_->async_send_request(request2);

      timer_->cancel();

      return;
    }

    std::cout << t << std::endl;

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
    pub_full_state_->publish(msg);
  }

  std::unique_ptr<SequenceControllerExperiment> ex;
  
  rclcpp::Publisher<FullState>::SharedPtr pub_full_state_;
  rclcpp::Subscription<NamedPoseArray>::SharedPtr sub_poses_;
  rclcpp::Client<Takeoff>::SharedPtr client_takeoff_;
  rclcpp::Client<Land>::SharedPtr client_land_;
  rclcpp::Client<NotifySetpointsStop>::SharedPtr client_notify_setpoints_stop_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::thread rai_thread_;

  Eigen::Affine3d pose_gate_;
  Eigen::Vector3d position_uav_;
  std::mutex mutex_mocap_; // protects pose_gate_ and position_uav_
  std::chrono::steady_clock::time_point spline_start_;
};

//===========================================================================

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rai::initCmdLine(argc, argv);

  //  rnd.clockSeed();
  rnd.seed(1);

  rclcpp::spin(std::make_shared<DemoNode>());
  rclcpp::shutdown();

  LOG(0) << " === bye bye ===\n used parameters:\n"
         << rai::getParameters()() << '\n';

  return 0;
}
