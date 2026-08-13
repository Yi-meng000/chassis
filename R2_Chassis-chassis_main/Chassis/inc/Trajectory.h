#ifndef __TRAJECTORY_H__
#define __TRAJECTORY_H__

#include "wheelTrain.h"
#include "mathFunc.h"

typedef enum
{
    Bezier, // 贝塞尔
    Bspline, 
    Bspline3, 
    Bspline5, //
}TrajMode; //轨迹类型
typedef struct _Bpoint
{
    s16 x;
    s16 y;
    float t;
}Bpoint;
Bpoint LinearInterpolation(Bpoint p1,Bpoint p2,float t);
Bpoint calBezierPoint(Bpoint *ctrl_points,int rank,float t);
Bpoint BspLine2D(Bpoint *ctrlpoint,float *node,int ctrlpointlen,int k,int Nodelen,float u);

#endif /* __TRAJECTORY_H__ */
