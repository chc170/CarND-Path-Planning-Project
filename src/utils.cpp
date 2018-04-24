#include "utils.h"
#include "math.h"


int Utils::d2lane(double d)
{
    if (d < 4)
    {
        return 0;
    }

    if (d < 8)
    {
        return 1;
    }

    return 2;
}

double Utils::deg2rad(double x)
{
    return x * M_PI / 180;
}

double Utils::rad2deg(double x)
{
    return x * 180 / M_PI;
}

double Utils::distance(double x1, double y1, double x2, double y2)
{
    return sqrt((x2-x1) * (x2-x1) + (y2-y1) * (y2-y1));
}
