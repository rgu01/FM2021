#pragma once
#include <stdlib.h>  /* malloc, free, rand, system */
#include <stdio.h>
#include <math.h>
#include <vector>

#define M_PI 3.14159265358979323846264338327950288
#define FLOWFIELD_SCOPE 20
#define GAMA 0.5
//#define DEBUG
//#define BIT16
#define BIT32

using namespace std;

#define X 0
#define Y 1
#define IPI 3142
#define AVSCALAR 4
#define AVVECTOR 1
#define SPACEDIMENSION 2
#define POSITIONID 0
#define WID 0
#define HEADID 1
#define ACCID 2
#define SPEEDID 3
#define CONSTRATE -1
#define NOTDIMENSIONAL -1
#define MAXACC 7
#define MINACC -10
#define MAXSPEED 28
#define MINSPEED 0
#define MAXW 393
#define MINW -393
#define ANGULARSCALE 1000
#define NUMOBS 1

/*#ifdef BIT16
typedef struct
{
    int16_t x;
    int16_t y;
}Point;
typedef struct
{
    int16_t w;
    int16_t heading;
    int16_t acc;
    int16_t speed;
}VehicleState;
typedef struct
{
    int16_t numVertices;
    Point vertices[10];
}StaticObstacle;
#endif*/

#ifdef BIT32
typedef struct
{
    int32_t x;
    int32_t y;
}Point;
typedef struct
{
    int32_t w;
    int32_t heading;
}RotationalState;
typedef struct
{
    int32_t acc;
    int32_t speed;
}LinearState;
typedef struct
{
    int32_t numVertices;
    Point vertices[10];
}StaticObstacle;
#endif

typedef struct
{
    double x;
    double y;
}Point2f;

typedef struct
{
    double w;
    double heading;
    double acc;
    double speed;
}VehicleState2f;

double cmax(double v1, double v2);
double cmin(double v1, double v2);
double cdot(Point p1, Point p2);
Point cminus(Point p1, Point p2);
Point cplus(Point p1, Point p2);
Point cmultiply(Point p1, double t);
Point2f fmultiply(Point2f p1, double t);
Point2f fplus(Point2f p1, Point2f p2);
double getDisPointToPoint(Point2f p1, Point2f p2);

/*#ifdef BIT16
int16_t getDisPointToLine(Point2f v, Point2f w, Point2f p, Point2f *projection);
int16_t isAhead(Point current, Point goal, Point obs);
extern "C" ALGOLIB_EXPORTS int16_t getProjection(int16_t value, int16_t angle, int16_t axis);
extern "C" ALGOLIB_EXPORTS int16_t getAngle(int16_t sX, int16_t sY, int16_t eX, int16_t eY, int16_t scale);
extern "C" ALGOLIB_EXPORTS int16_t getDisPointToPoint(int16_t sx, int16_t sy, int16_t ex, int16_t ey);
extern "C" ALGOLIB_EXPORTS int16_t getDisPointToLine(int16_t px, int16_t py, int16_t sx, int16_t sy, int16_t ex, int16_t ey);
extern "C" ALGOLIB_EXPORTS int16_t getValidRange(int16_t length, int16_t angle, int16_t scale);
extern "C" ALGOLIB_EXPORTS int16_t detectStaticObstacles(Point vehicle, int16_t range, StaticObstacle * sobs, int16_t obsNum);
#endif*/

#ifdef BIT32
int32_t getDisPointToLine(Point2f v, Point2f w, Point2f p, Point2f* projection);
int32_t isAhead(Point current, Point goal, Point obs);
extern "C" int32_t getProjection(int32_t value, int32_t angle, int32_t axis);
extern "C" int32_t getAngle(int32_t sX, int32_t sY, int32_t eX, int32_t eY, int32_t scale);
extern "C" int32_t getDisPointToPoint(int32_t sx, int32_t sy, int32_t ex, int32_t ey);
extern "C" int32_t getDisPointToLine(int32_t px, int32_t py, int32_t sx, int32_t sy, int32_t ex, int32_t ey);
extern "C" int32_t getValidRange(int32_t length, int32_t angle, int32_t scale);
extern "C" int32_t detectStaticObstacles(Point vehicle, int32_t range, StaticObstacle * sobs, int32_t obsNum);
#endif
