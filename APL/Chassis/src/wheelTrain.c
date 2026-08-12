#include "wheelTrain.h"
#include "mathFunc.h"

void Chassis_Init(CHASSIS *chassis)
{
    *chassis = (CHASSIS){0};

    chassis->wheel[FL].cosPhaseAngle = COS_ANGLE_FL; // -135
    chassis->wheel[FL].sinPhaseAngle = SIN_ANGLE_FL;

    chassis->wheel[FR].cosPhaseAngle = COS_ANGLE_FR; // 135
    chassis->wheel[FR].sinPhaseAngle = SIN_ANGLE_FR;

    chassis->wheel[BL].cosPhaseAngle = COS_ANGLE_BL; // -45
    chassis->wheel[BL].sinPhaseAngle = SIN_ANGLE_BL;

    chassis->wheel[BR].cosPhaseAngle = COS_ANGLE_BR; // 45
    chassis->wheel[BR].sinPhaseAngle = SIN_ANGLE_BR;
}
/**
 * @brief 保证最小转角
 *
 * @param wheel
 * @param targetAngle 角度制
 * @return int
 */
int wheelTurnMin(WHEEL *wheel, float targetAngle)
{
    float deltaAngle = DEG2RAD(targetAngle) - wheel->angleSetRad;
    int temp = floor((deltaAngle / PI) + 0.5f);

    wheel->angleSetRad += deltaAngle - PI * temp;

    wheel->angleSetDeg = RAD2DEG(wheel->angleSetRad);

    return powf(-1.f, temp);
}
/**
 * @brief 限速，将世界坐标系下的速度转换到车身坐标系下
 *
 * @param chassis
 */
void Chassis_carvelSet(CHASSIS *chassis)
{
    float AngleRealRad = DEG2RAD(chassis->ChassisPosReal.angle);
    float carVxSet = chassis->ChassisPosSet.vx * cosf(AngleRealRad) + chassis->ChassisPosSet.vy * sinf(AngleRealRad);
    float carVySet = chassis->ChassisPosSet.vy * cosf(AngleRealRad) - chassis->ChassisPosSet.vx * sinf(AngleRealRad);

    chassis->ChassisPosSet.vx = carVxSet;
    chassis->ChassisPosSet.vy = carVySet;

    chassis->ChassisPosSet.v = Modulo2d((vector2d){chassis->ChassisPosSet.vx, chassis->ChassisPosSet.vy});

    if (chassis->ChassisPosSet.v > CHASSIS_MANUAL_MAX_VELOCITY)
    {
        chassis->ChassisPosSet.vx = chassis->ChassisPosSet.vx * CHASSIS_MANUAL_MAX_VELOCITY / chassis->ChassisPosSet.v;
        chassis->ChassisPosSet.vy = chassis->ChassisPosSet.vy * CHASSIS_MANUAL_MAX_VELOCITY / chassis->ChassisPosSet.v;
        chassis->ChassisPosSet.v = CHASSIS_MANUAL_MAX_VELOCITY;
    }
    if (fabs(chassis->ChassisPosSet.w) > CHASSIS_MANUAL_MAX_ANGULAR_VELOCITY)
        chassis->ChassisPosSet.w = GetSign(chassis->ChassisPosSet.w) * CHASSIS_MANUAL_MAX_ANGULAR_VELOCITY;
}
/**
 * @brief 车速到轮速
 *
 * @param wheel
 * @param carVxSet m/s
 * @param carVyset m/s
 * @param carVw °/s
 */
void CalSingWheelSpeed(WHEEL *wheel, float carVxset, float carVyset, float carVw)
{
    wheel->VxSet = (carVxset + DEG2RAD(carVw) * WHEEL2CENTER * wheel->cosPhaseAngle) * carVel2RPM;
    wheel->VySet = (carVyset + DEG2RAD(carVw) * WHEEL2CENTER * wheel->sinPhaseAngle) * carVel2RPM;
    wheel->VSet = sqrtf(wheel->VxSet * wheel->VxSet + wheel->VySet * wheel->VySet);
    float aimAngle = RAD2DEG(atan2(wheel->VySet, wheel->VxSet));
    wheel->VSet *= wheelTurnMin(wheel, aimAngle);
}
/**
 * @brief 发送舵轮控制信号
 *
 * @param chassis
 */
// void sendCtrlMsg(CHASSIS *chassis)
//{
//     sendSteerAngle((s16)chassis->wheel[FL].angleSetDeg,(s16)chassis->wheel[FR].angleSetDeg,(s16)chassis->wheel[BL].angleSetDeg,(s16)chassis->wheel[BR].angleSetDeg);
//     sendDrivingSpeed((s16)(chassis->wheel[FL].VSet * 5.f),(s16)(chassis->wheel[FR].VSet * 5.f),(s16)(chassis->wheel[BL].VSet * 5.f),(s16)(chassis->wheel[BR].VSet * 5.f));

//}
/**
//  * @brief 十字差锁
//  *
//  * @param chassis 4
//  */
// void crossLock(CHASSIS *chassis)
// {
//     chassis->ChassisPosSet.v  = 0;
//     chassis->ChassisPosSet.vx = 0;
//     chassis->ChassisPosSet.vy = 0;
//     chassis->ChassisPosSet.w  = 0;
//     chassis->crossBrake = true;
// }
