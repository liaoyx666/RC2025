#include "Launch_Speed.h"
#include "math.h"

float Get_Launch_RPM(float d)
{
	float Speed;
	Speed = (g*d*d)/(2 * cos(a)) *
	(d * tan(a) + 76.72f - H - (k * d * d * sin(a))/(2*cos(a)*cos(a)));
    float RPM = sqrt(Speed) / 19;
	return RPM;
}

