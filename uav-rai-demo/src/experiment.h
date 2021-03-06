#pragma once

#include <BotOp/bot.h>
#include <BotOp/SecMPC.h>

//===========================================================================

struct SecMPC_Experiments{
  rai::Configuration& C;
  unique_ptr<SecMPC> mpc;
  unique_ptr<BotOp> bot;
  Metronome tic;
  uint stepCount = 0;

  std::mutex mutex_C_;

  KOMO& komo;
  double timeCost=1e0, ctrlCost=1e0;
  int subSeqStart=0, subSeqStop=-1;
  bool setNextWaypointTangent;

  SecMPC_Experiments(rai::Configuration& _C, KOMO& _komo, double cycleTime=.1, double timeCost=1e0, double ctrlCost=1e0, bool _setNextWaypointTangent=true)
    : C(_C),
      tic(cycleTime),
      komo(_komo),
      timeCost(timeCost), ctrlCost(ctrlCost), setNextWaypointTangent(_setNextWaypointTangent){
  }

  bool step();

  void selectSubSeq(int subSeqStart=0, int _subSeqStop=-1);
};

//===========================================================================

//helper
void randomWalkPosition(rai::Frame* f, arr& centerPos, arr& velocity, double rate=.001);
