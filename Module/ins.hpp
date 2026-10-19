#pragma once

#include <cmath>
#include <errno.h>
#include <stdint.h>
#include <zephyr/kernel.h>
#include "quaternion.hpp"
#include "type_math.hpp"

struct InsEulerAngles
{
    float roll_rad;
    float pitch_rad;
    float yaw_rad;
    float yaw_unwrapped_rad;
    uint32_t timestamp_ms;
};

class Ins
{
    public:
    void Init();

    bool Process(int16_t ax_raw, int16_t ay_raw, int16_t az_raw, int16_t gx_raw, int16_t gy_raw, int16_t gz_raw, float dt_seconds);

    void SetAxisRotation(const float rotation[3][3]);

    void ResetAxisRotation();

    bool GetEulerAngles(InsEulerAngles &angles);

    private:
    bool FiniteMatrix(const float matrix[3][3]);

    float Determinant(const float matrix[3][3]);

    bool IsRotationMatrix(const float matrix[3][3]);
    alg::attitude::QuaternionEKF estimator_{};
    alg::attitude::QuaternionEKF::State estimator_state_{};
    float axis_rotation_[3][3] = {
        {1.0f, 0.0f, 0.0f},
        {0.0f, 1.0f, 0.0f},
        {0.0f, 0.0f, 1.0f},
    };
    bool axis_rotation_identity_ = true;
    struct k_spinlock ins_lock_{};
    float startup_gyro_bias_[3] = {0.0f, 0.0f, 0.0f};
    uint32_t startup_bias_samples_ = 0;
    uint32_t last_update_ms_ = 0;
};

void ins_init();

bool ins_process(int16_t ax_raw, int16_t ay_raw, int16_t az_raw, int16_t gx_raw, int16_t gy_raw, int16_t gz_raw, float dt_seconds);

void ins_set_axis_rotation(const float rotation[3][3]);

void ins_reset_axis_rotation();

bool ins_get_euler_angles(InsEulerAngles &angles);
