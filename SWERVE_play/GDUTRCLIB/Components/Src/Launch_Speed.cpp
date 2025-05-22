#include "Launch_Speed.h"

double calculate_launch_speed(double angle_deg, double distance) {
    double angle_rad = DEG_TO_RAD(angle_deg);
    double h = HOOP_HEIGHT - LAUNCH_HEIGHT;
    double d = distance;

    double numerator = GRAVITY * d * d;
    double denominator = 2.0 * pow(cos(angle_rad), 2.0) * (d * tan(angle_rad) - h);

    double v = sqrt(numerator / denominator);
    v *= (1.0 + AIR_RESISTANCE_COEFF);  // 考虑空气阻力

    double rpm = v * MOTOR_POLE_PAIRS * 60.0 / (2.0 * M_PI * MOTOR_RADIUS);
    return rpm;
}



