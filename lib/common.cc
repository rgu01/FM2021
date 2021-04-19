
#include "common.h"

double getDisPointToPoint(Point2f p1, Point2f p2)
{
    double r = sqrt(pow((p1.x - p2.x), 2) + pow((p1.y - p2.y), 2));
    return r;
}

double cmax(double v1, double v2)
{
    double f = (v1 > v2) ? v1 : v2;
    return f;
}

double cmin(double v1, double v2)
{
    double f = (v1 < v2) ? v1 : v2;
    return f;
}

double cdot(Point p1, Point p2)
{
    float f = p1.x * p2.x + p1.y * p2.y;
    return f;
}

Point cminus(Point p1, Point p2)
{
    Point r = { p1.x - p2.x, p1.y - p2.y };
    return r;
}

Point cplus(Point p1, Point p2)
{
    Point r = { p1.x + p2.x, p1.y + p2.y };
    return r;
}

Point cmultiply(Point p1, double t)
{
    Point r = { p1.x * t, p1.y * t };
    return r;
}

Point2f fmultiply(Point2f p1, double t)
{
    Point2f r = { p1.x * t, p1.y * t };
    return r;
}

Point2f fplus(Point2f p1, Point2f p2)
{
    Point2f r = { p1.x + p2.x, p1.y + p2.y };
    return r;
}

Point2f fminus(Point2f p1, Point2f p2)
{
    Point2f r = { p1.x - p2.x, p1.y - p2.y };
    return r;
}

double fdot(Point2f p1, Point2f p2)
{
    float f = p1.x * p2.x + p1.y * p2.y;
    return f;
}


double PointToSegDist(double x, double y, double x1, double y1, double x2, double y2)
{
    double cross = (x2 - x1) * (x - x1) + (y2 - y1) * (y - y1);
    if (cross <= 0) return sqrt((x - x1) * (x - x1) + (y - y1) * (y - y1)); //projection is (x1,y1)

    double d2 = (x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1);
    if (cross >= d2) return sqrt((x - x2) * (x - x2) + (y - y2) * (y - y2)); //projection is (x2,y2)

    double r = cross / d2;
    double px = x1 + (x2 - x1) * r;
    double py = y1 + (y2 - y1) * r;
    return sqrt((x - px) * (x - px) + (py - y) * (py - y));
}

/*#ifdef BIT16
int16_t getProjection(int16_t value, int16_t angle, int16_t axis)
{
    double dAngle = ((double)angle) / ((double)ANGULARSCALE);
    double result;

    if (axis == X)
    {
        result = value * cos(dAngle);
    }
    else
    {
        result = value * sin(dAngle);
    }
    if (result < 1 && result > 0)
    {
        result = 1.0;
    }
    else if (result < 0 && result > -1)
    {
        result = -1.0;
    }

    return (int16_t)result;
}

int16_t getAngle(int16_t sX, int16_t sY, int16_t eX, int16_t eY, int16_t scale)
{
    double dAngle = atan2((eY - sY), (eX - sX));
    int16_t iAngle = (int16_t)(dAngle * scale);

    return iAngle;
}

int16_t getDisPointToPoint(int16_t x1, int16_t y1, int16_t x2, int16_t y2)
{
    double r = sqrt(pow((x1 - x2), 2) + pow((y1 - y2), 2));
    return (int)r;
}

int16_t getValidRange(int16_t length, int16_t angle, int16_t scale)
{
    double dAngle = ((double)angle) / ((double)scale);
    double result = sqrt(2) * length * sin(dAngle);

    return (int16_t)result;
}

int16_t getDisPointToLine(Point2f start, Point2f end, Point2f current, Point2f* projection)
{
    const double l2 = pow((start.x - end.x), 2) + pow((start.y - end.y), 2);
    if (l2 == 0.0) return getDisPointToPoint(current.x, current.y, start.x, start.y);
    const double t = fmax(0, fmin(1, fdot(fminus(current, start), fminus(end, start)) / l2));
    Point2f sf = { start.x, start.y }, ef = { end.x, end.y };
    *projection = fplus(sf, fmultiply(fminus(ef, sf), t));

    const int16_t d = getDisPointToPoint(current.x, current.y, projection->x, projection->y);
    //int d = PointToSegDist(current.x, current.y, start.x, start.y, end.x, end.y);

    return d;
}

int16_t getDisPointToLine(Point v, Point w, Point p) {
    // Return minimum distance between line segment vw and point p
    const double l2 = pow((v.x - w.x), 2) + pow((v.y - w.y), 2);       // i.e. |w-v|^2 -  avoid a sqrt
    if (l2 == 0.0) return getDisPointToPoint(p.x, p.y, v.x, v.y);   // v == w case
    // Consider the line extending the segment, parameterized as v + t (w - v).
    // We find projection of point p onto the line. 
    // It falls where t = [(p-v) . (w-v)] / |w-v|^2
    // We clamp t from [0,1] to handle points outside the segment vw.
    const double t = cmax(0, cmin(1, cdot(cminus(p, v), cminus(w, v)) / l2));
    const Point projection = cplus(v, cmultiply(cminus(w, v), t));  // Projection falls on the segment
    //printf("projection:%d,%d\n", projection.x, projection.y);
    const int16_t d = getDisPointToPoint(p.x, p.y, projection.x, projection.y);
    //printf("d:%d\n", d);

    return d;
}

int16_t getDisPointToLine(int16_t px, int16_t py, int16_t sx, int16_t sy, int16_t ex, int16_t ey)
{
    Point v = { sx, sy };
    Point w = { ex, ey };
    Point p = { px, py };
    int16_t result = getDisPointToLine(v, w, p);

    return (int16_t)result;
}

//if the projection of obs onto the segment between current and goal on the segment
int16_t isAhead(Point current, Point goal, Point obs)
{
    Point2f projection;
    Point2f fCurrent = { current.x, current.y };
    Point2f fGoal = { goal.x, goal.y };
    Point2f fObs = { obs.x, obs.y };
    double dis = getDisPointToLine(fCurrent, fGoal, fObs, &projection);

    if ((projection.x == current.x && projection.y == current.y) || (projection.x == goal.x && projection.y == goal.y))
    {
        return 0;
    }

    return 1;
}

int16_t detectStaticObstacles(Point vehicle, int16_t range, StaticObstacle* sobs, int16_t obsNum)
{
    int i = 0, j = 0, dis = 0, result = 0;

    for (i = 0; i < obsNum; i++)
    {
        for (j = 0; j < sobs[i].numVertices; j++)
        {
            if (j != sobs[i].numVertices - 1)
            {
                dis = getDisPointToLine(sobs[i].vertices[j], sobs[i].vertices[j + 1], vehicle);
            }
            else
            {
                dis = getDisPointToLine(sobs[i].vertices[j], sobs[i].vertices[0], vehicle);
            }
            result = (dis > range) ? 1 : 0;
        }
    }

    return 0;
}
#endif // BIT16*/

#ifdef BIT32
int32_t getProjection(int32_t value, int32_t angle, int32_t axis)
{
    double dAngle = ((double)angle) / ((double)ANGULARSCALE);
    double result;
    int upper = 0, lower = 0, iResult = 0;

    if (axis == X)
    {
        result = value * cos(dAngle);
    }
    else
    {
        result = value * sin(dAngle);
    }
    if (result < 1 && result > 0)
    {
        result = 1.0;
    }
    else if (result < 0 && result > -1)
    {
        result = -1.0;
    }
    lower = (int32_t)result;
    upper = lower + 1;
    if (upper - result > result - lower)
    {
        iResult = lower;
    }
    else
    {
        iResult = upper;
    }

    return iResult;
}

int32_t getAngle(int32_t sX, int32_t sY, int32_t eX, int32_t eY, int32_t scale)
{
    double dAngle = atan2((eY - sY), (eX - sX));
    int32_t iAngle = (int32_t)(dAngle * scale);

    return iAngle;
}

int32_t getDisPointToPoint(int32_t x1, int32_t y1, int32_t x2, int32_t y2)
{
    double r = sqrt(pow((x1 - x2), 2) + pow((y1 - y2), 2));
    return (int)r;
}

int32_t getValidRange(int32_t length, int32_t angle, int32_t scale)
{
    double dAngle = ((double)angle) / ((double)scale);
    double result = sqrt(2) * length * sin(dAngle);

    return (int32_t)result;
}

int32_t getDisPointToLine(Point2f start, Point2f end, Point2f current, Point2f* projection)
{
    const double l2 = pow((start.x - end.x), 2) + pow((start.y - end.y), 2);
    if (l2 == 0.0) return getDisPointToPoint(current.x, current.y, start.x, start.y);
    const double t = fmax(0, fmin(1, fdot(fminus(current, start), fminus(end, start)) / l2));
    Point2f sf = { start.x, start.y }, ef = { end.x, end.y };
    *projection = fplus(sf, fmultiply(fminus(ef, sf), t));

    const int32_t d = getDisPointToPoint(current.x, current.y, projection->x, projection->y);
    //int d = PointToSegDist(current.x, current.y, start.x, start.y, end.x, end.y);

    return d;
}

int32_t getDisPointToLine(Point v, Point w, Point p) {
    // Return minimum distance between line segment vw and point p
    const double l2 = pow((v.x - w.x), 2) + pow((v.y - w.y), 2);       // i.e. |w-v|^2 -  avoid a sqrt
    if (l2 == 0.0) return getDisPointToPoint(p.x, p.y, v.x, v.y);   // v == w case
    // Consider the line extending the segment, parameterized as v + t (w - v).
    // We find projection of point p onto the line. 
    // It falls where t = [(p-v) . (w-v)] / |w-v|^2
    // We clamp t from [0,1] to handle points outside the segment vw.
    const double t = cmax(0, cmin(1, cdot(cminus(p, v), cminus(w, v)) / l2));
    const Point projection = cplus(v, cmultiply(cminus(w, v), t));  // Projection falls on the segment
    //printf("projection:%d,%d\n", projection.x, projection.y);
    const int32_t d = getDisPointToPoint(p.x, p.y, projection.x, projection.y);
    //printf("d:%d\n", d);

    return d;
}

int32_t getDisPointToLine(int32_t px, int32_t py, int32_t sx, int32_t sy, int32_t ex, int32_t ey)
{
    Point v = { sx, sy };
    Point w = { ex, ey };
    Point p = { px, py };
    int32_t result = getDisPointToLine(v, w, p);

    return (int32_t)result;
}

//if the projection of obs onto the segment between current and goal on the segment
int32_t isAhead(Point current, Point goal, Point obs)
{
    Point2f projection;
    Point2f fCurrent = { current.x, current.y };
    Point2f fGoal = { goal.x, goal.y };
    Point2f fObs = { obs.x, obs.y };
    double dis = getDisPointToLine(fCurrent, fGoal, fObs, &projection);

    if ((projection.x == current.x && projection.y == current.y) || (projection.x == goal.x && projection.y == goal.y))
    {
        return 0;
    }

    return 1;
}

int32_t detectStaticObstacles(Point vehicle, int32_t range, StaticObstacle* sobs, int32_t obsNum)
{
    int i = 0, j = 0, dis = 0, result = 0;

    for (i = 0; i < obsNum; i++)
    {
        for (j = 0; j < sobs[i].numVertices; j++)
        {
            if (j != sobs[i].numVertices - 1)
            {
                dis = getDisPointToLine(sobs[i].vertices[j], sobs[i].vertices[j + 1], vehicle);
            }
            else
            {
                dis = getDisPointToLine(sobs[i].vertices[j], sobs[i].vertices[0], vehicle);
            }
            result = (dis > range) ? 1 : 0;
        }
    }

    return 0;
}
#endif // BIT32
