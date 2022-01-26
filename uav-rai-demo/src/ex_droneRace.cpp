#include "experiment.h"
#include <Kin/viewer.h>

void ex_droneRace(std::unique_ptr<SecMPC_Experiments>& ex){
  rai::Configuration C;
  C.addFile("droneRace.g");

  //-- add a mesh to allow drawing the spline plan
  rai::Mesh splineDisp;
  C.gl()->add(splineDisp);

  //-- define constraints
#if 0 //Wolfgang's multi waypoints
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
#else
  KOMO komo;
  komo.setModel(C, false);
  komo.setTiming(8., 1, 5., 1);
  komo.add_qControlObjective({}, 1, 1e-1);
  komo.addObjective({1.}, FS_positionDiff, {"drone", "target0_before"}, OT_eq, {1e1});
  komo.addObjective({2.}, FS_positionDiff, {"drone", "target0_shift"}, OT_eq, {1e1});
  komo.addObjective({3.}, FS_positionDiff, {"drone", "target1_shift"}, OT_eq, {1e1});
  komo.addObjective({4.}, FS_positionDiff, {"drone", "target2_shift"}, OT_eq, {1e1});
  komo.addObjective({5.}, FS_positionDiff, {"drone", "target3_shift"}, OT_eq, {1e1});
  komo.addObjective({6.}, FS_positionDiff, {"drone", "target0_shift"}, OT_eq, {1e1});
  komo.addObjective({7.}, FS_positionDiff, {"drone", "target1_shift"}, OT_eq, {1e1});
  komo.addObjective({8.}, FS_position, {"drone"}, OT_eq, {1e1}, {0,-.5, 1.});
#endif

  arrA targetCen(4), targetVel(4);

  ex = std::make_unique<SecMPC_Experiments>(C, komo, .1, 1e0, 1, false); //LAST ARGUMENT: NO AUTO-TANGENTS!

  ex->step();
  ex->mpc->tauCutoff = .1;
#if 0 //possibility to hard-code the tangents (requires true above) - but much less dynamic
  arr& T=ex->mpc->timingMPC.tangents;
  T.resize(komo.T-1, 3);
  for(uint k=0;k<komo.T-1;k+=2){
    T[k+0] = {1., -1., 0.};
    T[k+1] = {1., 1., 0.};
  }
  ex->mpc->setNextWaypointTangent=false;
#endif

  while(ex->step()){
    if(ex->mpc->timingMPC.phase==5){ //hard code endless loop by phase backtracking
      ex->mpc->timingMPC.update_setPhase(1);
    }

    //simulate wiggling the gates
#ifdef MARC_BUILD
    for(uint g=0;g<2;g++){
      rai::Frame *target = C[STRING("target"<<g)];
      randomWalkPosition(target, targetCen(g), targetVel(g), .003);
    }
#endif

    //update the spline display during stepping
    rai::CubicSpline tmpS;
    ex->mpc->timingMPC.getCubicSpline(tmpS, ex->bot->get_q(), ex->bot->get_qDot());
    splineDisp.V = tmpS.eval(range(0., tmpS.times.last(), 100));
    splineDisp.makeLineStrip();
  }
}

