#include "Trajectory.h"

Bpoint LinearInterpolation(Bpoint p1,Bpoint p2,float t)
{
    Bpoint pr;
    p1.x *= (1-t);
    p1.y *= (1-t);
    p2.x *= t;
    p2.y *= t;
    pr.x = p1.x + p2.x;
    pr.y = p1.y + p2.y;
    return pr;
}
Bpoint calBezierPoint(Bpoint *ctrl_points,int rank,float t)
{
    Bpoint result;
    if(rank == 0)
        return ctrl_points[0];
    result = LinearInterpolation(calBezierPoint(&ctrl_points[0],rank - 1, t) , calBezierPoint(&ctrl_points[1], rank - 1,t),t);
    return result;
}
Bpoint BspLine2D(Bpoint *ctrlpoint,float *node,int ctrlpointlen,int k,int Nodelen,float u)
{
    ;
}
