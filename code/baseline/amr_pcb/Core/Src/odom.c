/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : odom.c
  * @brief          : Differential-drive odometry state and math.
  ******************************************************************************
  */
/* USER CODE END Header */

#include "app_config.h"
#include "diagnostics.h"
#include "motion_control.h"
#include "odom.h"

#include <math.h>
#include <string.h>

DriveOdom_t g_odom;

float Odom_NormalizeAngleRad(float a)
{
    while (a > ODOM_PI) {
        a -= 2.0f * ODOM_PI;
    }

    while (a < -ODOM_PI) {
        a += 2.0f * ODOM_PI;
    }

    return a;
}

void Odom_ResetFromRawPositions(int32_t left_pos, int32_t right_pos)
{
    memset(&g_odom, 0, sizeof(g_odom));
    g_odom.initialized = true;
    g_odom.left_zero = left_pos;
    g_odom.right_zero = right_pos;
    g_odom.left_last = left_pos;
    g_odom.right_last = right_pos;

    uart_printf("ODOM RESET left_zero=%ld right_zero=%ld\r\n",
                (long)g_odom.left_zero,
                (long)g_odom.right_zero);
}

void Odom_UpdateFromRawPositions(int32_t left_pos, int32_t right_pos, bool print)
{
    int32_t left_raw_delta = (int32_t)((uint32_t)left_pos - (uint32_t)g_odom.left_last);
    int32_t right_raw_delta = (int32_t)((uint32_t)right_pos - (uint32_t)g_odom.right_last);
    int32_t left_delta = Motion_ApplyMotorForwardSignI32(LEFT_NODE_ID,
                                                        left_raw_delta);
    int32_t right_delta = Motion_ApplyMotorForwardSignI32(RIGHT_NODE_ID,
                                                         right_raw_delta);
    float left_mm = Motion_MotorDeltaCountsToWheelDistanceMm(LEFT_NODE_ID,
                                                             left_raw_delta);
    float right_mm = Motion_MotorDeltaCountsToWheelDistanceMm(RIGHT_NODE_ID,
                                                              right_raw_delta);
    float ds = (left_mm + right_mm) * 0.5f;
    float dtheta = (right_mm - left_mm) / WHEEL_BASE_MM;
    float theta_mid = g_odom.theta_rad + (dtheta * 0.5f);

    g_odom.left_last = left_pos;
    g_odom.right_last = right_pos;
    g_odom.total_left_mm += left_mm;
    g_odom.total_right_mm += right_mm;
    g_odom.x_mm += ds * cosf(theta_mid);
    g_odom.y_mm += ds * sinf(theta_mid);
    g_odom.theta_rad = Odom_NormalizeAngleRad(g_odom.theta_rad + dtheta);

    if (print) {
        uart_printf("ODOM raw_pos L=%ld R=%ld zero L=%ld R=%ld\r\n",
                    (long)left_pos,
                    (long)right_pos,
                    (long)g_odom.left_zero,
                    (long)g_odom.right_zero);
        uart_printf("ODOM delta_counts raw L=%ld R=%ld logical L=%ld R=%ld\r\n",
                    (long)left_raw_delta,
                    (long)right_raw_delta,
                    (long)left_delta,
                    (long)right_delta);
        uart_printf("ODOM delta_mm L=%ld R=%ld\r\n",
                    (long)left_mm,
                    (long)right_mm);
        uart_printf("ODOM pose x=%ld mm y=%ld mm theta=%ld deg\r\n",
                    (long)g_odom.x_mm,
                    (long)g_odom.y_mm,
                    (long)(g_odom.theta_rad * 180.0f / ODOM_PI));
        uart_printf("ODOM total L=%ld mm R=%ld mm\r\n",
                    (long)g_odom.total_left_mm,
                    (long)g_odom.total_right_mm);
    }
}

void Odom_UpdateFromWheelVelocities(float left_velocity_mm_s,
                                    float right_velocity_mm_s,
                                    uint32_t dt_ms,
                                    bool print)
{
    float dt_s = (float)dt_ms / 1000.0f;
    float left_mm = left_velocity_mm_s * dt_s;
    float right_mm = right_velocity_mm_s * dt_s;
    float ds = (left_mm + right_mm) * 0.5f;
    float dtheta = (right_mm - left_mm) / WHEEL_BASE_MM;
    float theta_mid;

    if (!g_odom.initialized) {
        memset(&g_odom, 0, sizeof(g_odom));
        g_odom.initialized = true;
    }
    theta_mid = g_odom.theta_rad + (dtheta * 0.5f);
    g_odom.total_left_mm += left_mm;
    g_odom.total_right_mm += right_mm;
    g_odom.x_mm += ds * cosf(theta_mid);
    g_odom.y_mm += ds * sinf(theta_mid);
    g_odom.theta_rad = Odom_NormalizeAngleRad(g_odom.theta_rad + dtheta);

    if (print) {
        uart_printf(
            "ODOM velocity delta_mm L=%ld R=%ld dt_ms=%lu\r\n",
            (long)left_mm, (long)right_mm, (unsigned long)dt_ms);
    }
}
