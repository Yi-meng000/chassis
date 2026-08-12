#ifndef __CHASSISRUN_H__
#define __CHASSISRUN_H__

#include "Trajectory.h"
#include "chassisPid.h"
#include "Trajset.h"

extern uint8_t trajMarker;
extern bool *side_traj;
void TrajctoryParam_Init(void); 
void Trajctory_CtrlPointReverse(TrajParam *trajparam);
void chassis_SlopeClimb(CHASSIS *chassis);
#endif /* __CHASSISRUN_H__ */
