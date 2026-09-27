#ifndef SENSOR_FUSION_STUDENT_H
#define SENSOR_FUSION_STUDENT_H

#include <stdint.h>

/*
 * SENSOR FUSION BAN SINH VIEN
 * Phan cung: STM32F411 + MPU6050 + 2 encoder quadrature.
 * File nay khong phu thuoc HAL, nen de hoc, de test va de tai su dung.
 */

typedef enum
{
    SF_STUDENT_CALIBRATING = 0,
    SF_STUDENT_READY,
    SF_STUDENT_FAULT
} SFStudent_Status_t;

typedef enum
{
    SF_FAULT_NONE        = 0U,
    SF_FAULT_BAD_CONFIG  = (1U << 0),
    SF_FAULT_BAD_DT      = (1U << 1),
    SF_FAULT_COVARIANCE  = (1U << 2)
} SFStudent_Fault_t;

typedef struct
{
    /* Chuong 4, 10: tan so mau va thoi gian thuc. */
    float sample_hz;

    /* Chuong 5: thong so calibration accelerometer. */
    float accel_bias_mps2[3];
    float accel_scale[3];

    /* Chuong 11: anh xa truc cam bien sang truc than xe.
     * axis_index = {0,1,2}, axis_sign = {1,1,1} neu hai he truc trung nhau. */
    uint8_t axis_index[3];
    int8_t axis_sign[3];

    /* Chuong 12, 17: thong so hai banh va encoder. */
    float wheel_radius_left_m;
    float wheel_radius_right_m;
    float wheel_track_m;
    float encoder_cpr_left;
    float encoder_cpr_right;
    int8_t encoder_sign_left;
    int8_t encoder_sign_right;

    /* Chuong 6: nhieu gyro, random walk bias va nhieu goc accel. */
    float gyro_noise_rad_s_sqrt_hz;
    float gyro_bias_rw_rad_s2_sqrt_hz;
    float accel_angle_r_rad2;

    /* Chuong 7, 8: NIS, adaptive R va gating. */
    float adaptive_r_lambda;
    float nis_gate;
    uint16_t force_accept_after;

    /* Chuong 17: he so nhieu odometry phu thuoc quang duong. */
    float odom_a1;
    float odom_a2;
    float odom_a3;
    float odom_a4;

    /* Chuong 5: calibration gyro khi khoi dong. */
    uint16_t calibration_samples;
    float calibration_max_gyro_std_rad_s;
    float calibration_max_accel_error_mps2;
} SFStudent_Config_t;

typedef struct
{
    SFStudent_Status_t status;
    uint32_t fault_flags;

    float roll_rad;
    float pitch_rad;
    float yaw_rad;
    float gyro_rad_s[3];
    float gyro_bias_rad_s[3];

    float x_m;
    float y_m;
    float distance_m;
    float speed_mps;
    float wheel_speed_left_mps;
    float wheel_speed_right_mps;
    float wheel_yaw_rate_rad_s;
    float odom_covariance[3][3];

    float accel_norm_mps2;
    float temperature_c;
    float nis_roll;
    float nis_pitch;
    float nis_heading;
    uint32_t accel_update_accepted;
    uint32_t accel_update_rejected;
    uint32_t heading_update_accepted;
    uint32_t heading_update_rejected;

    uint16_t calibration_count;
    uint32_t calibration_restart_count;
} SFStudent_Output_t;

void SFStudent_DefaultConfig(SFStudent_Config_t *config);
uint8_t SFStudent_Init(const SFStudent_Config_t *config);

/* raw14 la 14 byte doc lien tuc tu thanh ghi MPU6050 ACCEL_XOUT_H (0x3B). */
void SFStudent_ProcessRawMPU(const uint8_t raw14[14],
                             uint16_t encoder_left,
                             uint16_t encoder_right,
                             float dt_s);

void SFStudent_GetOutput(SFStudent_Output_t *output);

#endif
