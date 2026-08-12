#ifndef __MATCH_H__
#define __MATCH_H__

#include "Sekiro.h"


typedef enum
{
    MatchInit,
    MatchZone1,
    MatchZone2,
    MatchZone3,
} matchstage;

extern uint8_t MatchPhase;

void Match_Init(SEKIRO *sekiro);
Task_State Sekiro_MartialClub(SEKIRO *sekiro);
Task_State Sekiro_MFCross(SEKIRO *sekiro);
Task_State Sekiro_Tictactoe(SEKIRO *sekiro);
Task_State Sekiro_SkillTictactoe(SEKIRO *sekiro);
Task_State Sekrio_MEILINCross_NoRoutes(SEKIRO *sekiro);
void Sekiro_ShinobiExecution(SEKIRO *sekiro);
void Sekiro_ShadowDiesTwice(SEKIRO *sekiro);
#endif /* __MATCH_H__ */
