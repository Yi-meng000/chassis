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
    // float carVxSet = chassis->ChassisPosSet.vx * cosf(AngleRealRad) + chassis->ChassisPosSet.vy * sinf(AngleRealRad);
    // float carVySet = chassis->ChassisPosSet.vy * cosf(AngleRealRad) - chassis->ChassisPosSet.vx * sinf(AngleRealRad);
    float carVxSet = chassis->ChassisPosSet.vx;
    float carVySet = chassis->ChassisPosSet.vy;

    chassis->ChassisPosSet.vx = carVxSet;
    chassis->ChassisPosSet.vy = carVySet;

    chassis->ChassisPosSet.v = Modulo2d((vector2d){chassis->ChassisPosSet.vx, chassis->ChassisPosSet.vy});

    // if (chassis->ChassisPosSet.v > CHASSIS_MANUAL_MAX_VELOCITY)
    // {
    //     chassis->ChassisPosSet.vx = chassis->ChassisPosSet.vx * CHASSIS_MANUAL_MAX_VELOCITY / chassis->ChassisPosSet.v;
    //     chassis->ChassisPosSet.vy = chassis->ChassisPosSet.vy * CHASSIS_MANUAL_MAX_VELOCITY / chassis->ChassisPosSet.v;
    //     chassis->ChassisPosSet.v = CHASSIS_MANUAL_MAX_VELOCITY;
    // }
    // if (fabs(chassis->ChassisPosSet.w) > CHASSIS_MANUAL_MAX_ANGULAR_VELOCITY)
    //     chassis->ChassisPosSet.w = GetSign(chassis->ChassisPosSet.w) * CHASSIS_MANUAL_MAX_ANGULAR_VELOCITY;
}

/*
 * Reconstruct the body-frame velocity from the four measured wheel speeds
 * (RPM) and steering angles.  The wheel equation is:
 *
 *   wheel_speed = cos(theta) * vx + sin(theta) * vy + k * w
 *
 * where w is rad/s and k is the tangential lever arm projected onto the
 * wheel direction.  A small diagonal damping term keeps the estimate usable
 * when all steering wheels are parallel, where one velocity component is
 * unobservable.
 */
void Chassis_UpdateMeasuredVelocity(CHASSIS *chassis)
{
    const float wheel_rpm_to_mps = (2.0f * PI * CHASSIS_ODOM_WHEEL_RADIUS_M) / 60.0f;
    const float damping = 1.0e-4f;
    float h00 = damping, h01 = 0.0f, h02 = 0.0f;
    float h11 = damping, h12 = 0.0f, h22 = damping;
    float g0 = 0.0f, g1 = 0.0f, g2 = 0.0f;
    float c00, c01, c02, c11, c12, c22, determinant;
    float vx, vy, w_rad;

    for (uint8_t i = 0U; i < 4U; ++i)
    {
        const WHEEL *wheel = &chassis->wheel[i];
        const float steer_rad = DEG2RAD(wheel->SteerMotorValueReal.angle);
        const float direction_x = cosf(steer_rad);
        const float direction_y = sinf(steer_rad);
        const float yaw_gain = CHASSIS_ODOM_WHEEL2CENTER_M *
                               (wheel->cosPhaseAngle * direction_x +
                                wheel->sinPhaseAngle * direction_y);
        const float wheel_speed = wheel->DriveMotorValueReal.speed * wheel_rpm_to_mps;

        h00 += direction_x * direction_x;
        h01 += direction_x * direction_y;
        h02 += direction_x * yaw_gain;
        h11 += direction_y * direction_y;
        h12 += direction_y * yaw_gain;
        h22 += yaw_gain * yaw_gain;

        g0 += direction_x * wheel_speed;
        g1 += direction_y * wheel_speed;
        g2 += yaw_gain * wheel_speed;
    }

    c00 = h11 * h22 - h12 * h12;
    c01 = h02 * h12 - h01 * h22;
    c02 = h01 * h12 - h02 * h11;
    c11 = h00 * h22 - h02 * h02;
    c12 = h01 * h02 - h00 * h12;
    c22 = h00 * h11 - h01 * h01;
    determinant = h00 * c00 + h01 * c01 + h02 * c02;

    if (fabsf(determinant) < 1.0e-9f)
        return;

    vx = (c00 * g0 + c01 * g1 + c02 * g2) / determinant;
    vy = (c01 * g0 + c11 * g1 + c12 * g2) / determinant;
    w_rad = (c02 * g0 + c12 * g1 + c22 * g2) / determinant;

    chassis->ChassisPosReal.vx = vx;
    chassis->ChassisPosReal.vy = vy;
    chassis->ChassisPosReal.v = sqrtf(vx * vx + vy * vy);
    chassis->ChassisPosReal.w = RAD2DEG(w_rad);
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
