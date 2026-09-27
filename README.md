# IMU + Wheel-Encoder Sensor Fusion for a Ground Robot (STM32F411)

> **Status: Work in progress.** The fusion library compiles cleanly and behaves as expected in a small PC smoke test.
> There is no unit-test file in this repository yet, and nothing has been tested on a real robot.

This is a HAL-independent C library that estimates roll, pitch, yaw and 2D pose (x, y) of a differential-drive robot.
Inputs are an MPU6050 IMU and two quadrature wheel encoders, sampled at 200 Hz.

## Algorithms

- **Two tilt Kalman filters** (roll and pitch). Each is a reduced error-state filter with state `[angle, gyro_bias]`:
  - gyro-driven prediction with a discretized process noise (dt, dt², dt³ terms)
  - accelerometer-angle update
  - injection into angle and bias
- **Robust update:** adaptive measurement noise that grows when ‖a‖ deviates from g, a χ² NIS gate (10.83, 1 DoF) and forced acceptance after N consecutive rejects
- **Heading:** gyro-z integration fused with the encoder yaw increment through an NIS test. The filter trusts the gyro when wheel slip is detected.
- **Odometry:** int16 counter differencing (wrap-safe), midpoint integration, and pose covariance `P = F P Fᵀ + G Q Gᵀ` with noise growing with |Δs| and |Δθ|
- **Boot calibration:** gyro bias from stationary samples (restarts if the robot moves), axis remapping sensor → body
- **Safety:** dt guard, covariance checks, and a status output (CALIBRATING / READY / FAULT) with fault flags

Yaw and position drift over time, because there is no magnetometer, camera or GNSS.

## Repository layout

```
Core/Inc/sensor_fusion_student.h   API and configuration struct
Core/Src/sensor_fusion_student.c   implementation (no HAL dependency)
integration/student_main_user_code.txt   code blocks to paste into CubeMX USER CODE sections of main.c
                                         (I2C1 DMA read, TIM2 200 Hz tick, TIM3/TIM4 encoder mode, DWT timestamps)
docs/chapter_to_student_code.txt   map from theory chapters to functions (in Vietnamese)
```

## How to use

1. Create an STM32F411 CubeMX project with I2C1 + DMA RX, TIM2 at 200 Hz (update interrupt), TIM3/TIM4 in encoder mode, and optionally IWDG.
2. Add the `.h`/`.c` files to `Core/Inc` and `Core/Src`.
3. Paste each block from `integration/student_main_user_code.txt` into the matching `USER CODE` section. Do not copy the `MX_..._Init()` functions.
4. Set the robot-specific values in `SFStudent_Config_t`: accel bias/scale, axis map, wheel radii, wheel track, encoder CPR and signs.
5. Keep the robot still for about 2 s at power-up. Only enable the motors when `status == SF_STUDENT_READY`.

API:

```c
SFStudent_DefaultConfig(&cfg);
SFStudent_Init(&cfg);
SFStudent_ProcessRawMPU(raw14, enc_left, enc_right, dt_s);   // every 5 ms
SFStudent_GetOutput(&out);
```

## Results

Re-run on 2026-09-26 with GCC 15.2:

- `gcc -std=c11 -Wall -Wextra -Werror -pedantic -c sensor_fusion_student.c`: no warnings.
- A throw-away smoke test (not included here) gave these results with the default configuration:
  - 1000 stationary samples: status READY, roll = pitch = yaw = 0.0000 rad
  - simulated 1 m straight drive: x = 1.000 m, y = 0.000 m
  - a 0.5 s dt: status FAULT with the `SF_FAULT_BAD_DT` flag

## Not done yet

- A proper unit-test file (turning, wheel slip, covariance growth)
- Tests on a real robot, and identification of noise parameters from real data

## About this code

This code was written with an AI assistant at my request. I defined the problem and the requirements.
The verification results above were re-run on my own machine.

## License

MIT. See [LICENSE](LICENSE). This is research and learning code with no warranty; it is not certified for safety-critical use.
