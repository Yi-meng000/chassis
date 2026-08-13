#include "wheelTrain.h"
#include "mathFunc.h"

CHASSIS Chassis;
CHASSIS_RUNMODE ChassisRun = RUN_NORMAL;
THRESHOLD Threshold;

void Chassis_Init(CHASSIS *chassis)
{
    *chassis = (CHASSIS){0};

    chassis->wheel[FL].cosPhaseAngle = COS_ANGLE_FL; // -135
    chassis->wheel[FL].sinPhaseAngle = SIN_ANGLE_FL;
    chassis->wheel[FL].crossAngle   =  45.f;

    chassis->wheel[FR].cosPhaseAngle = COS_ANGLE_FR; // 135
	chassis->wheel[FR].sinPhaseAngle = SIN_ANGLE_FR;
    chassis->wheel[FR].crossAngle   =  -45.f;

	chassis->wheel[BL].cosPhaseAngle = COS_ANGLE_BL; // -45
	chassis->wheel[BL].sinPhaseAngle = SIN_ANGLE_BL;
    chassis->wheel[BL].crossAngle   =  -45.f;

	chassis->wheel[BR].cosPhaseAngle = COS_ANGLE_BR; // 45
	chassis->wheel[BR].sinPhaseAngle = SIN_ANGLE_BR;
    chassis->wheel[BL].crossAngle   =  45.f;
    for(int i = 0 ; i < 4 ; i++)
    {
        #if ZMDR_CHASSIS
            chassis->wheel[i].DriveMotorValueReal.speed   = &Zmotor[i + 4].valReal.speed;
            chassis->wheel[i].DriveMotorValueReal.current = &Zmotor[i + 4].valReal.current;
            chassis->wheel[i].SteerMotorValueReal.angle   = &Zmotor[i].valReal.angle;
            chassis->wheel[i].SteerMotorValueReal.current = &Zmotor[i].valReal.current;
            #if SteeringWheel
                chassis->wheel[i].steerMotor = Zmotor + i;
                chassis->wheel[i].wheelMotor = Zmotor + i + 4;
            #else
                chassis->wheel[i].wheelMotor = Zmotor + i;
            #endif

        #endif
        #if VESC_CHASSIS
            chassis->wheel[i].DriveMotorValueReal.speed = &Vescmotor[i].valReal.speed;
            chassis->wheel[i].DriveMotorValueReal.current = &Vescmotor[i].valReal.current;
            chassis->wheel[i].wheelMotor = Vescmotor + i;
        #endif
        #if DJ_CHASSIS
            chassis->wheel[i].DriveMotorValueReal.speed = &DJmotor[i].valNow.speed;
            chassis->wheel[i].DriveMotorValueReal.current = &DJmotor[i].valNow.Current_A;
            chassis->wheel[i].wheelMotor = &DJmotor[i];
        #endif

    }

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

    return powf(-1.f,temp);
}
/**
 * @brief 限速，将世界坐标系下的速度转换到车身坐标系下 
 * 
 * @param chassis 
 */
void chassisSpeed2Wheel(CHASSIS *chassis)
{
    chassis->ChassisPosSet.v = Modulo2d((vector2d){chassis->ChassisPosSet.vx,chassis->ChassisPosSet.vy});

    if(chassis->ChassisPosSet.v > CHASSIS_MANUAL_MAX_VELOCITY)
    {
        chassis->ChassisPosSet.vx = chassis->ChassisPosSet.vx * CHASSIS_MANUAL_MAX_VELOCITY / chassis->ChassisPosSet.v;
        chassis->ChassisPosSet.vy = chassis->ChassisPosSet.vy * CHASSIS_MANUAL_MAX_VELOCITY / chassis->ChassisPosSet.v;
        chassis->ChassisPosSet.v = CHASSIS_MANUAL_MAX_VELOCITY;
    }
    if(fabs(chassis->ChassisPosSet.w) > CHASSIS_MANUAL_MAX_ANGULAR_VELOCITY) 
        chassis->ChassisPosSet.w = GetSign(chassis->ChassisPosSet.w) * CHASSIS_MANUAL_MAX_ANGULAR_VELOCITY;
    float AngleRealRad = DEG2RAD(chassis->ChassisPosReal.angle);
    float carVxSet = chassis->ChassisPosSet.vx * cosf(AngleRealRad) + chassis->ChassisPosSet.vy * sinf(AngleRealRad);
    float carVySet = chassis->ChassisPosSet.vy * cosf(AngleRealRad) - chassis->ChassisPosSet.vx * sinf(AngleRealRad);
    //默认无传感器AngleRealRad = 0即为在车身坐标系下的速度
		if(ChassisRun == RUN_CROSSBRAKE)
        crossLock(chassis);
		else
		{
			for(int i = 0 ; i < 4 ;i++)
					CalSingWheelSpeed(&chassis->wheel[i],carVxSet,carVySet,chassis->ChassisPosSet.w);
		}
}
/**
 * @brief 车速到轮速
 * 
 * @param wheel 
 * @param carVxSet m/s
 * @param carVyset m/s
 * @param carVw °/s
 */
void CalSingWheelSpeed(WHEEL *wheel ,float carVxset,float carVyset ,float carVw)
{
    wheel->VxSet = (carVxset + DEG2RAD(carVw) * WHEEL2CENTER * wheel->cosPhaseAngle) * carVel2RPM;
    wheel->VySet = (carVyset + DEG2RAD(carVw) * WHEEL2CENTER * wheel->sinPhaseAngle) * carVel2RPM;
    wheel->VSet = sqrtf(wheel->VxSet * wheel->VxSet + wheel->VySet * wheel->VySet);
    float aimAngle = RAD2DEG(atan2f(wheel->VySet,wheel->VxSet));

    wheel->VSet *= (wheelTurnMin(wheel,aimAngle));
    if(ChassisRun != RUN_NORMAL)
    {
        wheel->VxSet = 0;
        wheel->VySet = 0;
        wheel->VSet  = 0;    
    }
}

void Onmi_CalSingWheelSpeed(WHEEL *wheel,float carVxset,float carVyset ,float carVw)
{
    wheel->VSet = (wheel->cosPhaseAngle * carVxset + wheel->sinPhaseAngle * carVyset + WHEEL2CENTER * DEG2RAD(carVw) ) * carVel2RPM;
}
bool SteerTurn_Detect(CHASSIS *chassis)
{
		static int tmp = 0;
    for(int i = 0 ; i < 4 ; i++)
    {
        float deltaAngle = chassis->wheel[i].angleSetDeg - chassis->wheel[i].steerMotor->valReal.angle;
        //int temp = floor((deltaAngle / 180.f) + 0.5f);
        if(fabs(deltaAngle) > 5.f)
				{
					if(tmp++ > 15)
					{
						ZdriveSet(Zmotor[i].valSetNow.angle,i+1,PosIn);
						return true;
					}
          return false;
				}
    }

    return true;
}
/**
 * @brief 十字差锁
 * 
 * @param chassis 4
 */
void crossLock(CHASSIS *chassis)
{
    
	wheelTurnMin(&chassis->wheel[FL], 45);
	wheelTurnMin(&chassis->wheel[FR], -45);
	wheelTurnMin(&chassis->wheel[BL], -45);
	wheelTurnMin(&chassis->wheel[BR], 45);
    for(int i = 0 ; i < 4 ;i++)
        chassis->wheel[i].VSet = 0;
}
