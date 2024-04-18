#include "Kinematics.h"

void Kinematics::set_motor_param(uint8_t id, uint16_t reducation_ratio, uint16_t pulse_ration, float wheel_diameter)
{
    motor_param_[id].id = id;   // 设置电机ID
    motor_param_[id].reducation_ratio = reducation_ratio;   // 设置减速比
    motor_param_[id].pulse_ration = pulse_ration;   // 设置脉冲比
    motor_param_[id].wheel_diameter = wheel_diameter;   // 设置车轮直径
    motor_param_[id].per_pulse_distance = (wheel_diameter * PI) / (reducation_ratio * pulse_ration);   // 每个脉冲对应行驶距离
    motor_param_[id].speed_factor = (1000 * 1000) * (wheel_diameter * PI) / (reducation_ratio * pulse_ration);   // 计算速度因子
    Serial.printf("init motor param %d: %f=%f*PI/(%d*%d) speed_factor=%d\n", id, motor_param_[id].per_pulse_distance, wheel_diameter, reducation_ratio, pulse_ration, motor_param_[id].speed_factor);   // 打印调试信息
}

void Kinematics::set_kinematic_param(float wheel_distance)
{
    wheel_distance_ = wheel_distance;   // 设置轮间距离
}

void Kinematics::update_motor_ticks(uint64_t current_time, int32_t motor_tick1, int32_t motor_tick2)
{

    uint32_t dt = current_time - motor_param_[0].last_update_time;   // 计算时间差
    int32_t dtick1 = motor_tick1 - motor_param_[0].last_encoder_tick;   // 计算电机1脉冲差
    int32_t dtick2 = motor_tick2 - motor_param_[1].last_encoder_tick;   // 计算电机2脉冲差
    // 轮子速度计算
    motor_param_[0].motor_speed = dtick1 * (motor_param_[0].speed_factor / dt);   // 计算电机1轮子速度
    motor_param_[1].motor_speed = dtick2 * (motor_param_[1].speed_factor / dt);   // 计算电机2轮子速度

    motor_param_[0].last_encoder_tick = motor_tick1;   // 更新电机1上一次的脉冲计数
    motor_param_[1].last_encoder_tick = motor_tick2;   // 更新电机2上一次的脉冲计数
    motor_param_[0].last_update_time = current_time;   // 更新电机1上一次更新时间
    motor_param_[1].last_update_time = current_time;   // 更新电机2上一次更新时间
}

void Kinematics::kinematic_inverse(float linear_speed, float angular_speed, float &out_wheel1_speed, float &out_wheel2_speed)
{
}

void Kinematics::kinematic_forward(float wheel1_speed, float wheel2_speed, float &linear_speed, float &angular_speed)
{
    linear_speed = (wheel1_speed + wheel2_speed) / 2.0;   // 计算线速度
    angular_speed = (wheel2_speed - wheel1_speed) / wheel_distance_;   // 计算角速度
}

float Kinematics::motor_speed(uint8_t id)
{
    return motor_param_[id].motor_speed; // 返回指定id的轮子速度
}
