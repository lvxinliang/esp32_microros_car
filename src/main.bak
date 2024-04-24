#include <Arduino.h>
#include <micro_ros_platformio.h>    // 包含用于 ESP32 的 micro-ROS PlatformIO 库
#include <WiFi.h>                    // 包含 ESP32 的 WiFi 库
#include <rcl/rcl.h>                 // 包含 ROS 客户端库 (RCL)
#include <rclc/rclc.h>               // 包含用于 C 的 ROS 客户端库 (RCLC)
#include <rclc/executor.h>           // 包含 RCLC 执行程序库，用于执行订阅和发布
#include <geometry_msgs/msg/twist.h> // 包含 ROS2 geometry_msgs/Twist 消息类型
#include <Esp32PcntEncoder.h>        // 包含用于计数电机编码器脉冲的 ESP32 PCNT 编码器库
#include <Esp32McpwmMotor.h>         // 包含使用 ESP32 的 MCPWM 硬件模块控制 DC 电机的 ESP32 MCPWM 电机库
#include <PidController.h>           // 包含 PID 控制器库，用于实现 PID 控制

Esp32PcntEncoder encoders[2];      // 创建一个长度为 2 的 ESP32 PCNT 编码器数组
rclc_executor_t executor;          // 创建一个 RCLC 执行程序对象，用于处理订阅和发布
rclc_support_t support;            // 创建一个 RCLC 支持对象，用于管理 ROS2 上下文和节点
rcl_allocator_t allocator;         // 创建一个 RCL 分配器对象，用于分配内存
rcl_node_t node;                   // 创建一个 RCL 节点对象，用于此基于 ESP32 的机器人小车
rcl_subscription_t subscriber;     // 创建一个 RCL 订阅对象，用于订阅 ROS2 消息
geometry_msgs__msg__Twist sub_msg; // 创建一个 ROS2 geometry_msgs/Twist 消息对象
Esp32McpwmMotor motor;             // 创建一个 ESP32 MCPWM 电机对象，用于控制 DC 电机
float out_motor_speed[2];          // 创建一个长度为 2 的浮点数数组，用于保存输出电机速度
float current_speeds[2];           // 创建一个长度为 2 的浮点数数组，用于保存当前电机速度
PidController pid_controller[2];   // 创建PidController的两个对象

void twist_callback(const void *msg_in)
{
    const geometry_msgs__msg__Twist *twist_msg = (const geometry_msgs__msg__Twist *)msg_in;
    float linear_x = twist_msg->linear.x;   // 获取 Twist 消息的线性 x 分量
    float angular_z = twist_msg->angular.z; // 获取 Twist 消息的角度 z 分量
    Serial.printf("linear_x: %f, angular_z: %f\n", linear_x, angular_z);
    if (linear_x == 0 && angular_z == 0) // 如果 Twist 消息没有速度命令
    {
        pid_controller[0].update_target(0); // 更新控制器的目标值
        pid_controller[1].update_target(0);
        motor.updateMotorSpeed(0, 0); // 停止第一个电机
        motor.updateMotorSpeed(1, 0); // 停止第二个电机
        return;                       // 退出函数
    }

    // 根据线速度和角速度控制两个电机的转速
    if (linear_x != 0)
    {
        pid_controller[0].update_target(linear_x * 1000); // 使用mm/s作为target
        pid_controller[1].update_target(-linear_x * 1000);
    }
    if (angular_z != 0)
    {
        pid_controller[0].update_target(0.1 * 1000 * (angular_z > 0 ? -1 : 1)); // 使用mm/s作为target
        pid_controller[1].update_target(0.1 * 1000 * (angular_z > 0 ? -1 : 1));
    }
}

// 这个函数是一个后台任务，负责设置和处理与 micro-ROS 代理的通信。
void microros_task(void *param)
{
    // 设置 micro-ROS 代理的 IP 地址。
    IPAddress agent_ip;
    agent_ip.fromString("192.168.1.30");

    // 使用 WiFi 网络和代理 IP 设置 micro-ROS 传输层。
    set_microros_wifi_transports("00V4", "z12345678", agent_ip, 8888);

    // 等待 2 秒，以便网络连接得到建立。
    delay(2000);

    // 设置 micro-ROS 支持结构、节点和订阅。
    allocator = rcl_get_default_allocator();
    rclc_support_init(&support, 0, NULL, &allocator);
    rclc_node_init_default(&node, "esp32_car", "", &support);
    rclc_subscription_init_default(
        &subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "/cmd_vel");

    // 设置 micro-ROS 执行器，并将订阅添加到其中。
    rclc_executor_init(&executor, &support.context, 1, &allocator);
    rclc_executor_add_subscription(&executor, &subscriber, &sub_msg, &twist_callback, ON_NEW_DATA);

    // 循环运行 micro-ROS 执行器以处理传入的消息。
    while (true)
    {
        delay(100);
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
    }
}

// 这个函数根据编码器读数更新两个轮子速度。
void update_speed()
{
    // 初始化静态变量以存储上一次更新时间和编码器读数。
    static uint64_t last_update_time = millis();
    static int64_t last_ticks[2];

    // 获取自上次更新以来的经过时间。
    uint64_t dt = millis() - last_update_time;
    if (dt == 0)
        return;

    // 获取当前的编码器读数并计算当前的速度。
    int32_t pt[2];
    pt[0] = encoders[0].getTicks() - last_ticks[0];
    pt[1] = encoders[1].getTicks() - last_ticks[1];
    current_speeds[0] = float(pt[0] * 0.13084936) / dt * 1000;
    current_speeds[1] = float(pt[1] * 0.13084936) / dt * 1000;

    // 更新上一次更新时间和编码器读数。
    last_update_time = millis();
    last_ticks[0] = encoders[0].getTicks();
    last_ticks[1] = encoders[1].getTicks();
}

void setup()
{
    // 初始化串口通信，波特率为115200
    Serial.begin(115200);
    // 将两个电机分别连接到引脚22、23和12、13上
    // motor.attachMotor(0, 22, 23);
    // motor.attachMotor(1, 12, 13);
    motor.attachMotor(0, 2, 4);  // 将电机0连接到引脚4和引脚2
    motor.attachMotor(1, 18, 19); // 将电机1连接到引脚18和引脚19
    // 在引脚32、33和26、25上初始化两个编码器
    encoders[0].init(0, 32, 33);
    encoders[1].init(1, 26, 25);
    // 初始化PID控制器的kp、ki和kd
    pid_controller[0].update_pid(0.825, 0.125, 0.0);
    pid_controller[1].update_pid(0.825, 0.125, 0.0);
    // 初始化PID控制器的最大输入输出，MPCNT大小范围在正负100之间
    pid_controller[0].out_limit(-200, 200);
    pid_controller[1].out_limit(-200, 200);

    // 在核心0上创建一个名为"microros_task"的任务，栈大小为10240
    xTaskCreatePinnedToCore(microros_task, "microros_task", 10240, NULL, 1, NULL, 0);
}

void loop()
{
    // 更新电机速度
    update_speed();
    // 计算最新的电机输出值
    out_motor_speed[0] = pid_controller[0].update(current_speeds[0]);
    out_motor_speed[1] = pid_controller[1].update(current_speeds[1]);
    // 更新电机0和电机1的速度值
    motor.updateMotorSpeed(0, out_motor_speed[0]);
    motor.updateMotorSpeed(1, out_motor_speed[1]);
    // 延迟10毫秒
    delay(10);
}