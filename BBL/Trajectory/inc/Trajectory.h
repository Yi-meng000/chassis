#ifndef __TRAJECTORY_H__
#define __TRAJECTORY_H__

#include "wheelTrain.h"
#include "MasterComm.h"
#include "mathFunc.h"
#include "includes.h"
typedef enum
{
    Bezier, // 贝塞尔
    Bspline,
    Bspline3,
    Bspline5, //
} TrajMode;   // 轨迹类型
typedef struct _Bpoint
{
    s16 x;
    s16 y;
    float t;
} Bpoint;
#define Bezier_MAX_RANK 8
#define BEZIER_N 50
extern float tbl_t[BEZIER_N + 1];
extern float tbl_s[BEZIER_N + 1];
Bpoint LinearInterpolation(Bpoint p1, Bpoint p2, float t);
Bpoint calBezierPoint(Bpoint *ctrl_points, int rank, float t);
Bpoint BspLine2D(Bpoint *ctrlpoint, float *node, int ctrlpointlen, int k, int nodelen, float u);
vector2d Vector_Point2vector(Bpoint point1, Bpoint point2);
vector2d BezierSpeed(Bpoint pPoints[], int rank, float t);
vector2d BezierAccel(Bpoint pPoints[], int rank, float t);
float arc_length_segment(float a, float b, Bpoint _point[], int rank);
float BezierLengthGauss(Bpoint _point[], int rank, int segments);
float BezierGetParamT(Bpoint points[], int rank, float S);
void BezierLengthTableUpdate(Bpoint points[], int rank);
float BezierCurvature(Bpoint points[], int rank, float t);
// float AdaptiveDeltaT(float curvature,float vel,float deltat_Tmax,float delta_Tmin);
#endif /* __TRAJECTORY_H__ */
