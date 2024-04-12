#ifndef __PIDCONTROLLER_H__ // 如果没有定义__PIDCONTROLLER_H__
#define __PIDCONTROLLER_H__ // 定义__PIDCONTROLLER_H__

class PidController
{ // 定义一个PID控制器类
public:
    PidController() = default;                   // 默认构造函数
    PidController(float kp, float ki, float kd); // 构造函数，传入kp、ki、kd

public:
    float target_;      // 目标值
    float out_mix_;     // 输出下限
    float out_max_;     // 输出上限
    float kp_;          // 比例系数
    float ki_;          // 积分系数
    float kd_;          // 微分系数
    float last_output_; // 上一次输出值
    // pid
    float error_sum_;           // 误差累积和
    float derror_;              // 误差变化率
    float error_pre_;           // 上上次误差
    float error_last_;          // 上一次误差
    float intergral_up_ = 2500; // 积分上限

public:
    float update(float control);                   // 更新输出值
    void reset();                                  // 重置PID控制器
    void update_pid(float kp, float ki, float kd); // 更新PID系数
    void update_target(float target);              // 更新目标值
    void out_limit(float out_mix, float out_max);  // 输出限制
};

#endif // __PIDCONTROLLER_H__ // 结束条件
