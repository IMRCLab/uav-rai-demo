#include "experiment.h"

  void ex_droneRace(std::unique_ptr<SequenceControllerExperiment>& ex){
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

  }

