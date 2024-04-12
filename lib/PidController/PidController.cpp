#include "PidController.h"
#include "Arduino.h"

PidController::PidController(float kp, float ki, float kd)
{
    reset();                // 初始化控制器
    update_pid(kp, ki, kd); // 更新PID参数
}

float PidController::update(float control)
{
    // 计算误差及其变化率
    float error = target_ - control; // 计算误差
    derror_ = error_last_ - error;   // 计算误差变化率
    error_last_ = error;

    // 计算积分项并进行积分限制
    error_sum_ += error;
    if (error_sum_ > intergral_up_)
        error_sum_ = intergral_up_;
    if (error_sum_ < -1 * intergral_up_)
        error_sum_ = -1 * intergral_up_;

    // 计算控制输出值
    float output = kp_ * error + ki_ * error_sum_ + kd_ * derror_;

    // 控制输出限幅
    if (output > out_max_)
        output = out_max_;
    if (output < out_mix_)
        output = out_mix_;

    // 保存上一次的控制输出值
    last_output_ = output;

    return output;
}

void PidController::update_target(float target)
{
    target_ = target; // 更新控制目标值
}

void PidController::update_pid(float kp, float ki, float kd)
{
    reset();  // 重置控制器状态
    kp_ = kp; // 更新比例项系数
    ki_ = ki; // 更新积分项系数
    kd_ = kd; // 更新微分项系数
}

void PidController::reset()
{
    // 重置控制器状态
    last_output_ = 0.0f; // 上一次的控制输出值
    target_ = 0.0f;      // 控制目标值
    out_mix_ = 0.0f;     // 控制输出最小值
    out_max_ = 0.0f;     // 控制输出最大值
    kp_ = 0.0f;          // 比例项系数
    ki_ = 0.0f;          // 积分项系数
    kd_ = 0.0f;          // 微分项系数
    error_sum_ = 0.0f;   // 误差累计值
    derror_ = 0.0f;      // 误差变化率
    error_last_ = 0.0f;  // 上一次的误差值
}

void PidController::out_limit(float out_mix, float out_max)
{
    out_mix_ = out_mix; // 控制输出最小值
    out_max_ = out_max; // 控制输出最大值
}
