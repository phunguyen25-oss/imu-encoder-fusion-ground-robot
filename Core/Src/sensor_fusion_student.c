#include "sensor_fusion_student.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define SF_PI               3.14159265358979323846f
#define SF_TWO_PI           (2.0f * SF_PI)
#define SF_GRAVITY          9.80665f
#define SF_ACCEL_LSB_PER_G  16384.0f       /* MPU6050, +-2 g */
#define SF_GYRO_LSB_PER_DPS 65.5f          /* MPU6050, +-500 deg/s */
#define SF_DEG_TO_RAD        (SF_PI / 180.0f)

/*
 * Chuong 1 + 6 + 16:
 * Moi truc nghieng dung mot Error-State Kalman Filter 2 trang thai:
 *     x = [goc, bias_gyro]^T
 * Day la ban rut gon cua ESKF: du doan sai so nho, cap nhat bang accel,
 * sau do "inject" truc tiep vao goc va bias. Phu hop cho robot mat dat.
 */
typedef struct
{
    float angle;
    float bias;
    float p00;
    float p01;
    float p10;
    float p11;
    float nis;
    uint16_t consecutive_rejects;
} TiltKF_t;

typedef struct
{
    SFStudent_Config_t cfg;
    SFStudent_Output_t out;
    TiltKF_t roll;
    TiltKF_t pitch;

    float gyro_z_bias;
    uint8_t encoder_started;
    uint16_t encoder_left_previous;
    uint16_t encoder_right_previous;

    double accel_sum[3];
    double gyro_sum[3];
    double gyro_square_sum[3];
} SFStudent_Context_t;

static SFStudent_Context_t sf;

static float wrap_pi(float angle)
{
    while (angle > SF_PI)  { angle -= SF_TWO_PI; }
    while (angle < -SF_PI) { angle += SF_TWO_PI; }
    return angle;
}

static int16_t decode_i16(uint8_t high, uint8_t low)
{
    return (int16_t)(((uint16_t)high << 8) | (uint16_t)low);
}

static uint8_t is_finite_positive(float value)
{
    return (uint8_t)(isfinite(value) && (value > 0.0f));
}

static uint8_t is_finite_nonnegative(float value)
{
    return (uint8_t)(isfinite(value) && (value >= 0.0f));
}

void SFStudent_DefaultConfig(SFStudent_Config_t *config)
{
    if (config == NULL) { return; }
    memset(config, 0, sizeof(*config));

    config->sample_hz = 200.0f;
    config->accel_scale[0] = 1.0f;
    config->accel_scale[1] = 1.0f;
    config->accel_scale[2] = 1.0f;

    config->axis_index[0] = 0U;
    config->axis_index[1] = 1U;
    config->axis_index[2] = 2U;
    config->axis_sign[0] = 1;
    config->axis_sign[1] = 1;
    config->axis_sign[2] = 1;

    config->wheel_radius_left_m = 0.0325f;
    config->wheel_radius_right_m = 0.0325f;
    config->wheel_track_m = 0.160f;
    config->encoder_cpr_left = 1560.0f;
    config->encoder_cpr_right = 1560.0f;
    config->encoder_sign_left = 1;
    config->encoder_sign_right = 1;

    config->gyro_noise_rad_s_sqrt_hz = 0.006f;
    config->gyro_bias_rw_rad_s2_sqrt_hz = 0.0002f;
    config->accel_angle_r_rad2 = 0.035f * 0.035f;
    config->adaptive_r_lambda = 3.0f;
    config->nis_gate = 10.83f;              /* Chi-square 1 bac tu do, 99.9%. */
    config->force_accept_after = 50U;

    config->odom_a1 = 0.02f;
    config->odom_a2 = 0.01f;
    config->odom_a3 = 0.02f;
    config->odom_a4 = 0.01f;

    config->calibration_samples = 400U;     /* 2 giay tai 200 Hz. */
    config->calibration_max_gyro_std_rad_s = 0.02f;
    config->calibration_max_accel_error_mps2 = 0.75f;
}

static uint8_t config_is_valid(const SFStudent_Config_t *c)
{
    uint8_t used[3] = {0U, 0U, 0U};
    uint8_t i;

    if ((c == NULL) || !is_finite_positive(c->sample_hz) ||
        !is_finite_positive(c->wheel_radius_left_m) ||
        !is_finite_positive(c->wheel_radius_right_m) ||
        !is_finite_positive(c->wheel_track_m) ||
        !is_finite_positive(c->encoder_cpr_left) ||
        !is_finite_positive(c->encoder_cpr_right) ||
        !is_finite_positive(c->gyro_noise_rad_s_sqrt_hz) ||
        !is_finite_nonnegative(c->gyro_bias_rw_rad_s2_sqrt_hz) ||
        !is_finite_positive(c->accel_angle_r_rad2) ||
        !is_finite_nonnegative(c->adaptive_r_lambda) ||
        !is_finite_positive(c->nis_gate) ||
        (c->force_accept_after == 0U) ||
        !is_finite_nonnegative(c->odom_a1) ||
        !is_finite_nonnegative(c->odom_a2) ||
        !is_finite_nonnegative(c->odom_a3) ||
        !is_finite_nonnegative(c->odom_a4) ||
        !is_finite_positive(c->calibration_max_gyro_std_rad_s) ||
        !is_finite_positive(c->calibration_max_accel_error_mps2) ||
        ((c->encoder_sign_left != 1) && (c->encoder_sign_left != -1)) ||
        ((c->encoder_sign_right != 1) && (c->encoder_sign_right != -1)) ||
        (c->calibration_samples < 20U))
    {
        return 0U;
    }

    for (i = 0U; i < 3U; ++i)
    {
        if ((c->axis_index[i] > 2U) || used[c->axis_index[i]] ||
            ((c->axis_sign[i] != 1) && (c->axis_sign[i] != -1)) ||
            !is_finite_positive(c->accel_scale[i]))
        {
            return 0U;
        }
        used[c->axis_index[i]] = 1U;
    }
    return 1U;
}

uint8_t SFStudent_Init(const SFStudent_Config_t *config)
{
    memset(&sf, 0, sizeof(sf));
    if (!config_is_valid(config))
    {
        sf.out.status = SF_STUDENT_FAULT;
        sf.out.fault_flags = SF_FAULT_BAD_CONFIG;
        return 0U;
    }

    sf.cfg = *config;
    sf.out.status = SF_STUDENT_CALIBRATING;
    return 1U;
}

/* Chuong 11: doi truc IMU sang quy uoc x tien, y trai, z len cua than xe. */
static void map_sensor_to_body(const float sensor[3], float body[3])
{
    uint8_t i;
    for (i = 0U; i < 3U; ++i)
    {
        body[i] = (float)sf.cfg.axis_sign[i] * sensor[sf.cfg.axis_index[i]];
    }
}

/* Chuong 4 + 5: doi raw sang SI, tru bias va sua scale accelerometer. */
static void decode_mpu6050(const uint8_t raw[14],
                           float accel_body[3],
                           float gyro_body[3],
                           float *temperature_c)
{
    float accel_sensor[3];
    float gyro_sensor[3];
    uint8_t i;

    for (i = 0U; i < 3U; ++i)
    {
        float accel_raw = (float)decode_i16(raw[2U * i], raw[2U * i + 1U]);
        accel_sensor[i] = (accel_raw / SF_ACCEL_LSB_PER_G) * SF_GRAVITY;
        accel_sensor[i] = (accel_sensor[i] - sf.cfg.accel_bias_mps2[i]) /
                          sf.cfg.accel_scale[i];
    }

    for (i = 0U; i < 3U; ++i)
    {
        float gyro_raw = (float)decode_i16(raw[8U + 2U * i], raw[9U + 2U * i]);
        gyro_sensor[i] = (gyro_raw / SF_GYRO_LSB_PER_DPS) * SF_DEG_TO_RAD;
    }

    map_sensor_to_body(accel_sensor, accel_body);
    map_sensor_to_body(gyro_sensor, gyro_body);
    *temperature_c = ((float)decode_i16(raw[6], raw[7]) / 340.0f) + 36.53f;
}

static void calibration_restart(void)
{
    if (sf.out.calibration_count > 0U)
    {
        sf.out.calibration_restart_count++;
    }
    memset(sf.accel_sum, 0, sizeof(sf.accel_sum));
    memset(sf.gyro_sum, 0, sizeof(sf.gyro_sum));
    memset(sf.gyro_square_sum, 0, sizeof(sf.gyro_square_sum));
    sf.out.calibration_count = 0U;
}

/* Chuong 5: dat xe dung yen, lay trung binh gyro de tim zero-bias. */
static void calibration_step(const float accel[3], const float gyro[3])
{
    float accel_norm = sqrtf(accel[0] * accel[0] + accel[1] * accel[1] +
                             accel[2] * accel[2]);
    float gyro_norm = sqrtf(gyro[0] * gyro[0] + gyro[1] * gyro[1] +
                            gyro[2] * gyro[2]);
    uint16_t n;
    uint8_t i;

    /* Khong cho calibration neu xe dang rung/chay. */
    if ((fabsf(accel_norm - SF_GRAVITY) > sf.cfg.calibration_max_accel_error_mps2) ||
        (gyro_norm > 0.15f))
    {
        calibration_restart();
        return;
    }

    for (i = 0U; i < 3U; ++i)
    {
        sf.accel_sum[i] += accel[i];
        sf.gyro_sum[i] += gyro[i];
        sf.gyro_square_sum[i] += (double)gyro[i] * (double)gyro[i];
    }
    sf.out.calibration_count++;

    if (sf.out.calibration_count < sf.cfg.calibration_samples) { return; }
    n = sf.out.calibration_count;

    for (i = 0U; i < 3U; ++i)
    {
        float mean = (float)(sf.gyro_sum[i] / (double)n);
        float mean_square = (float)(sf.gyro_square_sum[i] / (double)n);
        float variance = fmaxf(0.0f, mean_square - mean * mean);
        if (sqrtf(variance) > sf.cfg.calibration_max_gyro_std_rad_s)
        {
            calibration_restart();
            return;
        }
        sf.out.gyro_bias_rad_s[i] = mean;
    }

    {
        float ax = (float)(sf.accel_sum[0] / (double)n);
        float ay = (float)(sf.accel_sum[1] / (double)n);
        float az = (float)(sf.accel_sum[2] / (double)n);

        /* Chuong 2: Euler de hieu va hop le khi robot khong gan pitch +-90 do. */
        sf.roll.angle = atan2f(ay, az);
        sf.pitch.angle = atan2f(-ax, sqrtf(ay * ay + az * az));
    }

    sf.roll.bias = sf.out.gyro_bias_rad_s[0];
    sf.pitch.bias = sf.out.gyro_bias_rad_s[1];
    sf.gyro_z_bias = sf.out.gyro_bias_rad_s[2];

    sf.roll.p00 = sf.pitch.p00 = 0.0076f;       /* (5 deg)^2 */
    sf.roll.p11 = sf.pitch.p11 = 0.000076f;     /* (0.5 deg/s)^2 */
    sf.out.status = SF_STUDENT_READY;
}

/* Chuong 1 + 6: buoc predict cua KF voi Qd tinh theo dt thuc. */
static void tilt_predict(TiltKF_t *kf, float gyro, float dt)
{
    float qg2 = sf.cfg.gyro_noise_rad_s_sqrt_hz *
                sf.cfg.gyro_noise_rad_s_sqrt_hz;
    float qb2 = sf.cfg.gyro_bias_rw_rad_s2_sqrt_hz *
                sf.cfg.gyro_bias_rw_rad_s2_sqrt_hz;
    float dt2 = dt * dt;
    float dt3 = dt2 * dt;
    float old00 = kf->p00;
    float old01 = kf->p01;
    float old10 = kf->p10;
    float old11 = kf->p11;

    kf->angle = wrap_pi(kf->angle + (gyro - kf->bias) * dt);

    /* P = F P F' + Qd, F = [[1,-dt],[0,1]]. */
    kf->p00 = old00 - dt * (old01 + old10) + dt2 * old11 +
              qg2 * dt + qb2 * dt3 / 3.0f;
    kf->p01 = old01 - dt * old11 - qb2 * dt2 / 2.0f;
    kf->p10 = old10 - dt * old11 - qb2 * dt2 / 2.0f;
    kf->p11 = old11 + qb2 * dt;
}

/* Chuong 7 + 8: innovation, NIS, gating, adaptive R va Joseph update. */
static uint8_t tilt_update(TiltKF_t *kf, float measured_angle, float accel_error)
{
    float innovation = wrap_pi(measured_angle - kf->angle);
    float r = sf.cfg.accel_angle_r_rad2 *
              (1.0f + sf.cfg.adaptive_r_lambda * accel_error * accel_error);
    float s = kf->p00 + r;
    float k0;
    float k1;
    float old00;
    float old01;
    float old10;
    float old11;

    if (!is_finite_positive(s)) { return 0U; }
    kf->nis = innovation * innovation / s;

    if ((kf->nis > sf.cfg.nis_gate) &&
        (kf->consecutive_rejects < sf.cfg.force_accept_after))
    {
        kf->consecutive_rejects++;
        return 0U;
    }

    kf->consecutive_rejects = 0U;
    old00 = kf->p00;
    old01 = kf->p01;
    old10 = kf->p10;
    old11 = kf->p11;
    k0 = old00 / s;
    k1 = old10 / s;

    /* Chuong 16: inject phan sai so uoc luong vao trang thai danh dinh. */
    kf->angle = wrap_pi(kf->angle + k0 * innovation);
    kf->bias += k1 * innovation;

    /* Joseph form giup P on dinh hon tren float32 cua STM32F411. */
    kf->p00 = (1.0f - k0) * (1.0f - k0) * old00 +
              k0 * k0 * r;
    kf->p01 = (1.0f - k0) * (old01 - k1 * old00) + k0 * k1 * r;
    kf->p10 = (old10 - k1 * old00) * (1.0f - k0) + k1 * k0 * r;
    kf->p11 = old11 - k1 * old01 - k1 * old10 +
              k1 * k1 * old00 + k1 * k1 * r;

    /* Ep doi xung de giam sai so lam tron. */
    kf->p01 = kf->p10 = 0.5f * (kf->p01 + kf->p10);
    return 1U;
}

/* Chuong 17: cap nhat pose vi sai va truyen covariance P=FPF'+GQG'. */
static void odometry_update(uint16_t left, uint16_t right, float dtheta_gyro, float dt)
{
    float dl;
    float dr;
    float ds;
    float dtheta_encoder;
    float dtheta_used;
    float heading_innovation;
    float gyro_angle_variance;
    float encoder_angle_variance;
    float heading_s;
    float heading_k;
    float heading_mid;
    float var_s;
    float var_theta;
    float f[3][3] = {{1.0f, 0.0f, 0.0f},
                     {0.0f, 1.0f, 0.0f},
                     {0.0f, 0.0f, 1.0f}};
    float g[3][2];
    float temp[3][3] = {{0.0f}};
    float result[3][3] = {{0.0f}};
    uint8_t i;
    uint8_t j;
    uint8_t k;

    if (!sf.encoder_started)
    {
        sf.encoder_left_previous = left;
        sf.encoder_right_previous = right;
        sf.encoder_started = 1U;
        sf.out.yaw_rad = wrap_pi(sf.out.yaw_rad + dtheta_gyro);
        return;
    }

    /* Cast int16_t tu dong xu ly wrap counter 16-bit neu moi mau < 32768 count. */
    dl = (float)((int16_t)((uint16_t)(left - sf.encoder_left_previous)));
    dr = (float)((int16_t)((uint16_t)(right - sf.encoder_right_previous)));
    sf.encoder_left_previous = left;
    sf.encoder_right_previous = right;

    dl *= (float)sf.cfg.encoder_sign_left * SF_TWO_PI *
          sf.cfg.wheel_radius_left_m / sf.cfg.encoder_cpr_left;
    dr *= (float)sf.cfg.encoder_sign_right * SF_TWO_PI *
          sf.cfg.wheel_radius_right_m / sf.cfg.encoder_cpr_right;
    ds = 0.5f * (dl + dr);
    dtheta_encoder = (dr - dl) / sf.cfg.wheel_track_m;
    sf.out.wheel_yaw_rate_rad_s = dtheta_encoder / dt;

    /* Chuong 7 + 12: fusion delta-yaw gyro va encoder bang mot update vo huong.
     * Neu NIS lon, encoder co the dang truot nen tam thoi tin gyro. */
    gyro_angle_variance = sf.cfg.gyro_noise_rad_s_sqrt_hz *
                          sf.cfg.gyro_noise_rad_s_sqrt_hz * dt + 1.0e-7f;
    encoder_angle_variance = sf.cfg.odom_a1 * fabsf(dtheta_encoder) +
                             sf.cfg.odom_a2 * fabsf(ds) + 1.0e-6f;
    heading_innovation = wrap_pi(dtheta_encoder - dtheta_gyro);
    heading_s = gyro_angle_variance + encoder_angle_variance;
    sf.out.nis_heading = heading_innovation * heading_innovation / heading_s;

    if (sf.out.nis_heading <= sf.cfg.nis_gate)
    {
        heading_k = gyro_angle_variance / heading_s;
        dtheta_used = dtheta_gyro + heading_k * heading_innovation;
        sf.out.heading_update_accepted++;
    }
    else
    {
        dtheta_used = dtheta_gyro;
        sf.out.heading_update_rejected++;
    }

    heading_mid = sf.out.yaw_rad + 0.5f * dtheta_used;

    sf.out.x_m += ds * cosf(heading_mid);
    sf.out.y_m += ds * sinf(heading_mid);
    sf.out.yaw_rad = wrap_pi(sf.out.yaw_rad + dtheta_used);
    sf.out.distance_m += fabsf(ds);
    sf.out.speed_mps = ds / dt;
    sf.out.wheel_speed_left_mps = dl / dt;
    sf.out.wheel_speed_right_mps = dr / dt;

    f[0][2] = -ds * sinf(heading_mid);
    f[1][2] =  ds * cosf(heading_mid);
    g[0][0] = cosf(heading_mid);
    g[1][0] = sinf(heading_mid);
    g[2][0] = 0.0f;
    g[0][1] = -0.5f * ds * sinf(heading_mid);
    g[1][1] =  0.5f * ds * cosf(heading_mid);
    g[2][1] = 1.0f;

    var_s = sf.cfg.odom_a3 * fabsf(ds) + sf.cfg.odom_a4 * fabsf(dtheta_used);
    var_theta = sf.cfg.odom_a1 * fabsf(dtheta_used) + sf.cfg.odom_a2 * fabsf(ds);

    for (i = 0U; i < 3U; ++i)
        for (j = 0U; j < 3U; ++j)
            for (k = 0U; k < 3U; ++k)
                temp[i][j] += f[i][k] * sf.out.odom_covariance[k][j];

    for (i = 0U; i < 3U; ++i)
        for (j = 0U; j < 3U; ++j)
        {
            for (k = 0U; k < 3U; ++k)
                result[i][j] += temp[i][k] * f[j][k];
            result[i][j] += g[i][0] * var_s * g[j][0] +
                            g[i][1] * var_theta * g[j][1];
        }

    memcpy(sf.out.odom_covariance, result, sizeof(result));
}

static uint8_t covariance_ok(const TiltKF_t *kf)
{
    return (uint8_t)(isfinite(kf->p00) && isfinite(kf->p11) &&
                     (kf->p00 >= 0.0f) && (kf->p11 >= 0.0f));
}

void SFStudent_ProcessRawMPU(const uint8_t raw14[14],
                             uint16_t encoder_left,
                             uint16_t encoder_right,
                             float dt_s)
{
    float accel[3];
    float gyro[3];
    float accel_norm;
    float accel_error;
    float roll_acc;
    float pitch_acc;
    float dtheta;
    uint8_t accepted_roll;
    uint8_t accepted_pitch;

    if ((raw14 == NULL) || !isfinite(dt_s) || (dt_s < 0.001f) || (dt_s > 0.050f))
    {
        sf.out.status = SF_STUDENT_FAULT;
        sf.out.fault_flags |= SF_FAULT_BAD_DT;
        return;
    }

    decode_mpu6050(raw14, accel, gyro, &sf.out.temperature_c);
    accel_norm = sqrtf(accel[0] * accel[0] + accel[1] * accel[1] +
                       accel[2] * accel[2]);
    sf.out.accel_norm_mps2 = accel_norm;

    if (sf.out.status == SF_STUDENT_CALIBRATING)
    {
        calibration_step(accel, gyro);
        sf.out.roll_rad = sf.roll.angle;
        sf.out.pitch_rad = sf.pitch.angle;
        return;
    }
    if (sf.out.status != SF_STUDENT_READY) { return; }

    /* Chuong 9: tang attitude chay truoc, tang odometry dung yaw sau. */
    tilt_predict(&sf.roll, gyro[0], dt_s);
    tilt_predict(&sf.pitch, gyro[1], dt_s);

    roll_acc = atan2f(accel[1], accel[2]);
    pitch_acc = atan2f(-accel[0], sqrtf(accel[1] * accel[1] +
                                        accel[2] * accel[2]));
    accel_error = fabsf(accel_norm - SF_GRAVITY);
    accepted_roll = tilt_update(&sf.roll, roll_acc, accel_error);
    accepted_pitch = tilt_update(&sf.pitch, pitch_acc, accel_error);
    sf.out.accel_update_accepted += (uint32_t)accepted_roll + (uint32_t)accepted_pitch;
    sf.out.accel_update_rejected += (uint32_t)(!accepted_roll) + (uint32_t)(!accepted_pitch);

    sf.out.gyro_rad_s[0] = gyro[0] - sf.roll.bias;
    sf.out.gyro_rad_s[1] = gyro[1] - sf.pitch.bias;
    sf.out.gyro_rad_s[2] = gyro[2] - sf.gyro_z_bias;
    dtheta = sf.out.gyro_rad_s[2] * dt_s;
    odometry_update(encoder_left, encoder_right, dtheta, dt_s);

    sf.out.roll_rad = sf.roll.angle;
    sf.out.pitch_rad = sf.pitch.angle;
    sf.out.gyro_bias_rad_s[0] = sf.roll.bias;
    sf.out.gyro_bias_rad_s[1] = sf.pitch.bias;
    sf.out.gyro_bias_rad_s[2] = sf.gyro_z_bias;
    sf.out.nis_roll = sf.roll.nis;
    sf.out.nis_pitch = sf.pitch.nis;

    /* Chuong 3: khong co magnetometer/moc ngoai, yaw se troi theo thoi gian. */
    if (!covariance_ok(&sf.roll) || !covariance_ok(&sf.pitch))
    {
        sf.out.status = SF_STUDENT_FAULT;
        sf.out.fault_flags |= SF_FAULT_COVARIANCE;
    }
}

void SFStudent_GetOutput(SFStudent_Output_t *output)
{
    if (output != NULL) { *output = sf.out; }
}
