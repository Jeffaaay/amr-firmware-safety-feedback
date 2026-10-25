/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : odom.h
  * @brief          : Differential-drive odometry state and math.
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef ODOM_H
#define ODOM_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    bool initialized;
    int32_t left_zero;
    int32_t right_zero;
    int32_t left_last;
    int32_t right_last;

    float x_mm;
    float y_mm;
    float theta_rad;

    float total_left_mm;
    float total_right_mm;
} DriveOdom_t;

extern DriveOdom_t g_odom;

void Odom_ResetFromRawPositions(int32_t left_pos, int32_t right_pos);
void Odom_UpdateFromRawPositions(int32_t left_pos, int32_t right_pos, bool print);
void Odom_UpdateFromWheelVelocities(float left_velocity_mm_s,
                                    float right_velocity_mm_s,
                                    uint32_t dt_ms,
                                    bool print);
float Odom_NormalizeAngleRad(float a);

#ifdef __cplusplus
}
#endif

#endif /* ODOM_H */
