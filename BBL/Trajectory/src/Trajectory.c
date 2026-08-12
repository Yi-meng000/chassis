#include "Trajectory.h"

float tbl_t[BEZIER_N + 1] = {0};
float tbl_s[BEZIER_N + 1] = {0};

Bpoint LinearInterpolation(Bpoint p1, Bpoint p2, float t)
{
    Bpoint pr;
    p1.x *= (1 - t);
    p1.y *= (1 - t);
    p2.x *= t;
    p2.y *= t;
    pr.x = p1.x + p2.x;
    pr.y = p1.y + p2.y;
    return pr;
}
float BezierLengthGauss(Bpoint _point[], int rank, int segments)
{
    float length = 0;
    float dt = 1.f / segments;
    float a, b;
    for (int i = 0; i < segments; i++)
    {
        a = i * dt;
        b = (i + 1) * dt;
        length += arc_length_segment(a, b, _point, rank);
    }
    return length;
}
Bpoint calBezierPoint(Bpoint *ctrl_points, int rank, float t)
{
    Bpoint result;
    // if(rank == 0)
    //     return ctrl_points[0];
    // result = LinearInterpolation(calBezierPoint(&ctrl_points[0],rank - 1, t) , calBezierPoint(&ctrl_points[1], rank - 1,t),t);
    // return result;
    if (rank == 0)
        return ctrl_points[0];
    float tmp_x[Bezier_MAX_RANK + 1], tmp_y[Bezier_MAX_RANK + 1];
    for (int i = 0; i < rank + 1; i++)
    {
        tmp_x[i] = (float)ctrl_points[i].x;
        tmp_y[i] = (float)ctrl_points[i].y;
    }
    for (int j = 1; j <= rank; j++)
        for (int i = 0; i <= rank - j; i++)
        {
            tmp_x[i] = (1.0f - t) * tmp_x[i] + t * tmp_x[i + 1];
            tmp_y[i] = (1.0f - t) * tmp_y[i] + t * tmp_y[i + 1];
        }
    result.x = (s16)roundf(tmp_x[0]);
    result.y = (s16)roundf(tmp_y[0]);

    return result;
}
/**
 * @brief
 *
 * @param Node
 * @param i
 * @param k k为阶数 order = degree(次数) + 1
 * @param u
 * @return float
 */
float BspLineBasicFunction(float *Node, int i, int k, float u)
{
    float divide1, divide2;
    if (k == 1)
    {
        if (Node[i] <= u && u < Node[i + 1])
            return 1;
        else
            return 0;
    }
    else if (k >= 2)
    {
        if (fabs(Node[i + k - 1] - Node[i]) < 1e-6)
            divide1 = 0;
        else
            divide1 = (u - Node[i]) / (Node[i + k - 1] - Node[i]);
        if (fabs(Node[i + k] - Node[i + 1]) < 1e-6)
            divide2 = 0;
        else
            divide2 = (Node[i + k] - u) / (Node[i + k] - Node[i + 1]);
        return divide1 * BspLineBasicFunction(Node, i, k - 1, u) +
               divide2 * BspLineBasicFunction(Node, i + 1, k - 1, u);
    }
    else
        return -1;
}
Bpoint BspLine2D(Bpoint *ctrlpoint, float *node, int ctrlpointlen, int k, int nodelen, float u)
{
    Bpoint result = {0, 0};
    if (ctrlpointlen + k == nodelen)
    {
        for (int i = 0; i < ctrlpointlen; i++)
        {
            result.x += ctrlpoint[i].x * BspLineBasicFunction(node, i, k, u);
            result.y += ctrlpoint[i].y * BspLineBasicFunction(node, i, k, u);
        }
    }
    return result;
}
vector2d Vector_Point2vector(Bpoint point1, Bpoint point2)
{
    vector2d delta;
    delta.x = (float)(point2.x - point1.x);
    delta.y = (float)(point2.y - point1.y);
    return delta;
}

vector2d BezierSpeed(Bpoint pPoints[], int rank, float t)
{

    Bpoint ptmp = {0};
    vector2d result = {0};
    if (rank == 0)
        return result;
    // 构造一阶导数曲线的控制点（共 n 个）
    Bpoint qPoints[Bezier_MAX_RANK];
    for (int i = 0; i < rank; i++)
    {
        qPoints[i].x = rank * (pPoints[i + 1].x - pPoints[i].x);
        qPoints[i].y = rank * (pPoints[i + 1].y - pPoints[i].y);
    }
    ptmp = calBezierPoint(qPoints, rank - 1, t);
    result.x = ptmp.x;
    result.y = ptmp.y;

    return result;
}
vector2d BezierAccel(Bpoint pPoints[], int rank, float t)
{
    Bpoint ptmp = {0};
    vector2d result = {0};
    if (rank <= 1)
        return result;

    Bpoint qPoints[Bezier_MAX_RANK];
    for (int i = 0; i < rank - 1; i++)
    {
        qPoints[i].x = (rank - 1) * (rank * (pPoints[i + 2].x - pPoints[i + 1].x) - rank * (pPoints[i + 1].x - pPoints[i].x));
        qPoints[i].y = (rank - 1) * (rank * (pPoints[i + 2].y - pPoints[i + 1].y) - rank * (pPoints[i + 1].y - pPoints[i].y));
    }
    ptmp = calBezierPoint(qPoints, rank - 2, t);
    result.x = ptmp.x;
    result.y = ptmp.y;
    return result;
}

float BezierCurvature(Bpoint points[], int rank, float t)
{
    vector2d d1 = BezierSpeed(points, rank, t);
    vector2d d2 = BezierAccel(points, rank, t);

    float cross = Vector_CrossProduct(d1, d2);
    float dmod = Modulo2d(d1);

    if (dmod < 1e-6f)
        return 0.f;
    return fabsf(cross) / (dmod * dmod * dmod);
}

float arc_length_segment(float a, float b, Bpoint _point[], int rank)
{
    const float xg[3] = {-0.7746f, 0.0f, 0.7746f};
    const float wg[3] = {0.5556f, 0.8889f, 0.5556f};
    float half = (b - a) * 0.5f;
    float mid = (a + b) * 0.5f;
    float sum = 0.0f;
    for (int i = 0; i < 3; i++)
    {
        float t = mid + half * xg[i];
        float speed = Modulo2d(BezierSpeed(_point, rank, t)); // ||B'(t)||
        sum += wg[i] * speed;
    }
    return half * sum;
}
void BezierLengthTableUpdate(Bpoint points[], int rank)
{
    float length = 0.f;
    float dt = 1.f / BEZIER_N;
    tbl_t[0] = 0;
    tbl_s[0] = 0;
    for (int i = 0; i < BEZIER_N; i++)
    {
        float a = i * dt;
        float b = (i + 1) * dt;
        length += arc_length_segment(a, b, points, rank);
        tbl_t[i + 1] = b;
        tbl_s[i + 1] = length;
    }
}
int find_lower_index(float t)
{
    int low = 0, high = BEZIER_N;
    if (t <= tbl_t[low])
        return low;
    if (t >= tbl_t[high])
        return high;
    while (low <= high)
    {
        int mid = (low + high) / 2;
        if (tbl_t[mid] == t)
            return mid;
        else if (tbl_t[mid] < t)
            low = mid + 1;
        else
            high = mid - 1;
    }
    return high;
}

int find_lowerS_index(float S)
{
    int low = 0, high = BEZIER_N;
    if (S <= tbl_s[low])
        return low;
    if (S >= tbl_s[high])
        return high;
    while (low <= high)
    {
        int mid = (low + high) / 2;
        if (tbl_s[mid] == S)
            return mid;
        else if (tbl_s[mid] < S)
            low = mid + 1;
        else
            high = mid - 1;
    }
    return high;
}

float BezierGetParamT(Bpoint points[], int rank, float S)
{
    int index = find_lowerS_index(S);
    if (index >= BEZIER_N)
        index = BEZIER_N - 1;
    float t0 = tbl_t[index];
    float t1 = tbl_t[index + 1];
    float t = t0 + (t1 - t0) * (S - tbl_s[index]) / (tbl_s[index + 1] - tbl_s[index]);
    for (int i = 0; i < 2; i++)
    {
        // 找到表中 t 之前的最大已知点 (t_prev, S_prev)
        index = find_lower_index(t);
        float t_prev = tbl_t[index];
        float S_prev = tbl_s[index];

        // 计算从 t_prev 到 t 的弧长增量
        float delta_S = arc_length_segment(t_prev, t, points, rank);
        float S_t = S_prev + delta_S;

        // 计算速度模长
        float speed = Modulo2d(BezierSpeed(points, rank, t));

        // 牛顿修正
        float t_new = t - (S_t - S) / speed;

        // 限制在 [0,1] 内
        t = fmaxf(0.0f, fminf(1.0f, t_new));
    }
    // index = find_lower_index(t);
    // tbl_s[index] += arc_length_segment(tbl_t[index], t, points, rank);
    // tbl_t[index] = t;
    return t;
}

// float AdaptiveDeltaT(float curvature,float vel,float deltat_Tmax,float delta_Tmin)
// {

// }
