#include "experiment.h"

  bool SequenceControllerExperiment::step(ObjectiveL& phi){
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
