#ifndef __VECTOR_H__
#define __VECTOR_H__

typedef struct Vec2d
{
    float x,y;
}vector2d;
typedef struct Vec3d
{
    float x;
    float y;
    float z;
}vector3d;
float Modulo2d(vector2d v);
vector2d Vector_Add(vector2d v1,vector2d v2);
vector2d Vector_Minus(vector2d v1,vector2d v2);
vector2d Vector_MultiplyNum(vector2d v,float num);

#endif /* __VECTOR_H__ */

