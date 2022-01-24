#pragma once

#include <BotOp/bot.h>
#include <BotOp/SequenceController.h>

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

  bool step(ObjectiveL& phi);
};

