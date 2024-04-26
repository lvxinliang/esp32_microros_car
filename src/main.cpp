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
#include <Kinematics.h>              // 运动学相关实现
#include <nav_msgs/msg/odometry.h>
#include <micro_ros_utilities/string_utilities.h>
rcl_publisher_t odom_publisher;   // 用于发布机器人的里程计信息（Odom）
nav_msgs__msg__Odometry odom_msg; // 机器人的里程计信息

Esp32PcntEncoder encoders[2];      // 创建一个长度为 2 的 ESP32 PCNT 编码器数组
rclc_executor_t executor;          // 创建一个 RCLC 执行程序对象，用于处理订阅和发布
rclc_support_t support;            // 创建一个 RCLC 支持对象，用于管理 ROS2 上下文和节点
rcl_allocator_t allocator;         // 创建一个 RCL 分配器对象，用于分配内存
rcl_node_t node;                   // 创建一个 RCL 节点对象，用于此基于 ESP32 的机器人小车
rcl_subscription_t subscriber;     // 创建一个 RCL 订阅对象，用于订阅 ROS2 消息
geometry_msgs__msg__Twist sub_msg; // 创建一个 ROS2 geometry_msgs/Twist 消息对象
Esp32McpwmMotor motor;             // 创建一个 ESP32 MCPWM 电机对象，用于控制 DC 电机

float out_motor_speed[2];        // 创建一个长度为 2 的浮点数数组，用于保存输出电机速度
PidController pid_controller[2]; // 创建PidController的两个对象
Kinematics kinematics;           // 运动学相关对象

float last_motor_speed[2] = {0, 0};// 方便调试，保存上一次的电机速度
unsigned long previousMillis = 0;  // 保存上一次打印的时间
const long interval = 1000;        // 打印间隔时间

static float target_motor_speed0, target_motor_speed1;
float pid_p[2] = {0.625, 0.625}, pid_i[2] = {0.1, 0.1}, pid_d[2] = {1.25, 1.25};

void twist_callback(const void *msg_in)
{
    const geometry_msgs__msg__Twist *twist_msg = (const geometry_msgs__msg__Twist *)msg_in;
    float linear_x = twist_msg->linear.x;   // 获取 Twist 消息的线性 x 分量
    float angular_z = twist_msg->angular.z; // 获取 Twist 消息的角度 z 分量
    kinematics.kinematic_inverse(linear_x * 1000, angular_z, target_motor_speed0, target_motor_speed1);
    pid_controller[0].update_target(target_motor_speed0);
    pid_controller[1].update_target(target_motor_speed1);
    // Serial.printf("target_motor_speed0: %f, target_motor_speed1: %f\n", target_motor_speed0, target_motor_speed1);
}

// 这个函数是一个后台任务，负责设置和处理与 micro-ROS 代理的通信。
void microros_task(void *param)
{
    // 使用 micro_ros_string_utilities_set 函数设置到 odom_msg.header.frame_id 中
    odom_msg.header.frame_id = micro_ros_string_utilities_set(odom_msg.header.frame_id, "odom");
    odom_msg.child_frame_id = micro_ros_string_utilities_set(odom_msg.child_frame_id, "base_link");

    // 设置 micro-ROS 代理的 IP 地址。
    IPAddress agent_ip;
    agent_ip.fromString("192.168.1.30");

    // 使用 WiFi 网络和代理 IP 设置 micro-ROS 传输层。
    set_microros_wifi_transports((char *)"00V4", (char *)"z12345678", agent_ip, 8888);

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
    rclc_publisher_init_best_effort(
        &odom_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
        "odom");
    // 设置 micro-ROS 执行器，并将订阅添加到其中。
    rclc_executor_init(&executor, &support.context, 1, &allocator);
    rclc_executor_add_subscription(&executor, &subscriber, &sub_msg, &twist_callback, ON_NEW_DATA);

    // 循环运行 micro-ROS 执行器以处理传入的消息。
    while (true)
    {
        if (!rmw_uros_epoch_synchronized())
        {
            rmw_uros_sync_session(1000);
            // 如果时间同步成功，则将当前时间设置为MicroROS代理的时间，并输出调试信息。
            delay(10);
        }
        delay(100);
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
    }
}

void setup()
{
    // 初始化串口通信，波特率为115200
    Serial.begin(115200);
    // 将两个电机分别连接到引脚22、23和12、13上
    motor.attachMotor(0, 2, 4);   // 将电机0连接到引脚4和引脚2
    motor.attachMotor(1, 18, 19); // 将电机1连接到引脚18和引脚19
    // 在引脚32、33和26、25上初始化两个编码器
    encoders[0].init(0, 32, 33);
    encoders[1].init(1, 26, 25);
    // 初始化PID控制器的kp、ki和kd
    pid_controller[0].update_pid(pid_p[0], pid_i[0], pid_d[0]);
    pid_controller[1].update_pid(pid_p[1], pid_i[1], pid_d[1]);
    // 初始化PID控制器的最大输入输出，MPCNT大小范围在正负100之间
    pid_controller[0].out_limit(-500, 500);
    pid_controller[1].out_limit(-500, 500);

    // 设置运动学参数
    kinematics.set_motor_param(0, 30, 52, 65); // 15606/10/30 = 52
    kinematics.set_motor_param(1, 30, 52, 65);
    kinematics.set_kinematic_param(160);

    // 在核心0上创建一个名为"microros_task"的任务，栈大小为10240
    xTaskCreatePinnedToCore(microros_task, "microros_task", 10240, NULL, 1, NULL, 0);
}

void loop()
{
    static float out_motor_speed[2];
    static uint64_t last_update_info_time = millis();
    static float filtered_motor_speed[2] = {0.0, 0.0};
    float alpha = 0.4; // 调整这个值以改变滤波器的强度，范围是0-1

    kinematics.update_motor_ticks(micros(), encoders[0].getTicks(), encoders[1].getTicks());
    out_motor_speed[0] = pid_controller[0].update(kinematics.motor_speed(0));
    out_motor_speed[1] = pid_controller[1].update(kinematics.motor_speed(1));

    // 一阶滤波器
    filtered_motor_speed[0] = alpha * out_motor_speed[0] + (1 - alpha) * filtered_motor_speed[0];
    filtered_motor_speed[1] = alpha * out_motor_speed[1] + (1 - alpha) * filtered_motor_speed[1];

    // 使用滤波后的速度更新电机
    motor.updateMotorSpeed(0, filtered_motor_speed[0]);
    motor.updateMotorSpeed(1, filtered_motor_speed[1]);
    Serial.printf("data: %f,%f,%f\n", target_motor_speed0, kinematics.motor_speed(0), pid_controller[0].error_sum_);
    // Serial.printf("data: %f,%f,%f,%f\n", target_motor_speed0, kinematics.motor_speed(0), target_motor_speed1, kinematics.motor_speed(1));
    if (Serial.available()) {
        String input = Serial.readStringUntil('\n');
        sscanf(input.c_str(), "%f,%f,%f", &pid_p[0], &pid_i[0], &pid_d[0]);
        pid_p[1] = pid_p[0], pid_i[1] = pid_i[0], pid_d[1] = pid_d[0];
        pid_controller[0].update_pid(pid_p[0], pid_i[0], pid_d[0]);
        pid_controller[1].update_pid(pid_p[1], pid_i[1], pid_d[1]);

        pid_controller[0].out_limit(-500, 500);
        pid_controller[1].out_limit(-500, 500);
        Serial.printf("pid_p: %f, pid_i: %f, pid_d: %f\n", pid_p, pid_i, pid_d);
    }


    // unsigned long currentMillis = millis(); // 获取当前时间
    // if (currentMillis - previousMillis >= interval)
    // {                                   // 判断是否到达间隔时间
    //     previousMillis = currentMillis; // 记录上一次打印的时间
    //     float linear_speed, angle_speed;
    //     kinematics.kinematic_forward(kinematics.motor_speed(0), kinematics.motor_speed(1), linear_speed, angle_speed);
    //     // Serial.printf("[%ld] linear:%f angle:%f\n", currentMillis, linear_speed, angle_speed);                                      // 打印当前时间
    //     // Serial.printf("[%ld] x:%f y:%f yaml:%f\n", currentMillis, kinematics.odom().x, kinematics.odom().y, kinematics.odom().yaw); // 打印当前时间
    //     int64_t stamp = rmw_uros_epoch_millis();
    //     // 获取机器人的位置和速度信息，并将其存储在一个ROS消息（odom_msg）中
    //     odom_t odom = kinematics.odom();
    //     odom_msg.header.stamp.sec = static_cast<int32_t>(stamp / 1000); // 秒部分
    //     odom_msg.header.stamp.nanosec = static_cast<uint32_t>((stamp % 1000) * 1e6); // 纳秒部分
    //     odom_msg.pose.pose.position.x = odom.x;
    //     odom_msg.pose.pose.position.y = odom.y;
    //     odom_msg.pose.pose.orientation.w = odom.quaternion.w;
    //     odom_msg.pose.pose.orientation.x = odom.quaternion.x;
    //     odom_msg.pose.pose.orientation.y = odom.quaternion.y;
    //     odom_msg.pose.pose.orientation.z = odom.quaternion.z;

    //     odom_msg.twist.twist.angular.z = odom.angular_speed;
    //     odom_msg.twist.twist.linear.x = odom.linear_speed;

    //     rcl_ret_t ret = rcl_publish(&odom_publisher, &odom_msg, NULL);
    //     if (ret != RCL_RET_OK) {
    //         Serial.printf("Failed to publish odom message: %d\n", ret);
    //     }
    // }
    // 延迟10毫秒
    delay(10);
}
