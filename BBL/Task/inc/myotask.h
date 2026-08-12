#ifndef __MYOTASK_H__
#define __MYOTASK_H__

#include "tim.h"
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "includes.h"
#include "LED.h"
#include "chassisPid.h"
#include "ActcatorCom.h"
#include "sensorparam.h"
#include "IRQhandler.h"
#include "Cameracom.h"
#include "chassisRun.h"
#include "waveform.h"

void StartDefaultTask(void *argument);
void TaskUart(void *argument);
void TaskChassis(void *argument);
void SekiroTask(void *argument);
void Testtask(void *argument);
void TaskTrajctory(void *argument);
#endif /* __MYOTASK_H__ */
