#include "triangle.h" 
#include "MathLib.h"



//sin函数查表法,传入的是角度 
float sin_calculate(float angle)
{
    float sin_angle;

    if (angle >= 0.0f && angle < 90.0f)
        sin_angle = (Trigonometric_Functions[(int)(float_abs(angle) * 10.0f)] / 100.0f);
    else if (angle >= 90.0f && angle < 180.f)
        sin_angle = (Trigonometric_Functions[(int)(float_abs(180.0f - angle) * 10.0f)] / 100.0f);
    else if (angle >= -180.0f && angle < -90.f)
        sin_angle = -(Trigonometric_Functions[(int)(float_abs(180.0f + angle) * 10.0f)] / 100.0f);
    else if (angle >= -90.0f && angle < 0.f)
        sin_angle = -(Trigonometric_Functions[(int)(float_abs(180.0f - (180.0f + angle)) * 10.0f)] / 100.0f);
	else if (angle == 180.f)
		sin_angle = 0.0f;

    return sin_angle;
}

//cos函数查表法
float cos_calculate(float angle)
{
    float cos_angle;

    angle = float_abs(angle);

    if (angle >= 0.0f && angle < 90.0f)
        cos_angle = (Trigonometric_Functions[(int)(float_abs(90.0f - angle) * 10.0f)] / 100.0f);
    else if (angle >= 90.0f && angle < 180.f)
        cos_angle = -(Trigonometric_Functions[(int)(float_abs(angle - 90.0f) * 10.0f)] / 100.0f);
	else if (angle == 180.f)
		cos_angle = -1.0f;

    return cos_angle;
}

