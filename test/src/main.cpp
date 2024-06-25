#include <iostream>
#include <string>
#include <thread>
#include <mutex>
#include <atomic> 
#include <condition_variable> 
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "task_control_interface/msg/mcu_to_task.hpp"
#include "task_control_interface/msg/app_cmd.hpp"

using namespace task_control_interface::msg;

rclcpp::Node::SharedPtr g_node = nullptr;
AppCmd g_app_cmd_msg;
std::atomic_bool running(true);

void inputThread() {
  while (running) {
      std::string input;
      std::cout << "(模式控制) 手动模式请输入: mode=1; 自动模式请输入: mode=2" << std::endl;
      std::cout << "(急停控制) 急停请输入: emr=1; 急停恢复请输入: emr=2" << std::endl;
      std::cout << "(手动作业) 上装作业请输入:manual=1; 上装停止作业请输入:manual=2" << std::endl;
      std::cout << "(自动作业) 自动作业请输入: auto=1~4; 1-开始  2-暂停  3-继续  4-停止" << std::endl;
      std::cout << "(吸盘) 请输入: cup=0~1; 0-不放吸盘 1-放吸盘" << std::endl;
      std::cout << "(LED) 请输入: led=0~6; 0-关闭 1-绿灯常亮 2-绿灯闪亮 3-黄灯常亮 4-黄灯闪亮 5-红灯常亮 6-红灯闪亮" << std::endl;
      std::cout << "(蜂鸣器) 请输入: buzzer=0~3; 0-关闭 1-蜂鸣器一直响 2-蜂鸣器长响一声后关 3-蜂鸣器长响两声后关" << std::endl;
      std::cout << "(辊刷) 请输入: brush=0~2000; 辊刷速度(RPM)" << std::endl;
      std::cout << "请输入一个字符串: ";  
      std::getline(std::cin, input); // 使用getline读取整行输入
      // 检查输入是否为退出命令  
      if (input == "exit") {  
          running = false; // 退出循环
          continue;  
      }
      // 查找"="的位置
      size_t equalPos = input.find('=');  
      if (equalPos != std::string::npos) {  
          // 提取"="左侧的字符串  
          auto cmd = input.substr(0, equalPos);  
          // 提取"="右侧的字符串  
          auto val = input.substr(equalPos + 1);
          // 打印左右两侧的字符串  
          std::cout << "左侧的字符串: " << cmd << ", 右侧的字符串: " << val << std::endl;
          if (cmd == "mode") {
              g_app_cmd_msg.mode_cmd = std::stoi(val);
          } else if (cmd == "emr") {
              g_app_cmd_msg.emr_cmd = std::stoi(val);
          } else if (cmd == "manual") {
              g_app_cmd_msg.manual_cmd = std::stoi(val);
          } else if (cmd == "auto") {
              g_app_cmd_msg.auto_cmd = std::stoi(val);
          }else if (cmd == "cup") {
              g_app_cmd_msg.cup = std::stoi(val);
          } else if (cmd == "led") {
              g_app_cmd_msg.led = std::stoi(val);
          } else if (cmd == "buzzer") {
              g_app_cmd_msg.buzzer = std::stoi(val);
          } else if (cmd == "brush") {
              g_app_cmd_msg.brush = std::stoi(val);
          } else {}
      } else {
          std::cout << "输入的字符串中不包含'='字符" << std::endl;
          continue;
      }
  }

}

void mcu_to_task_callback(const McuToTask::SharedPtr msg) {
  std::cout << "subscribe topic /mcu_to_task: "
            << (msg->emergency_stop_status == 0 ? "急停接触" : "急停中") 
            << (msg->suction_cup_status == 0 ? "吸盘接触" : "吸盘吸住中")
            << (msg->motor_status == 0 ? "履带电机正常" : "履带电机异常")
            << (msg->brush_status == 0 ? "辊刷正常" : "辊刷异常")
            << (msg->drop_sign == 0 ? "防跌落未触发" : "防跌落触发")
            << "异常码：" << msg->exception_code << std::endl;

}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto g_node = rclcpp::Node::make_shared("app_simulator");
  auto app_cmd_publisher = g_node->create_publisher<AppCmd>("app_cmd", 10);
  auto mcu_to_task_subscriber = g_node->create_subscription<McuToTask>("mcu_to_task", 10, mcu_to_task_callback);
  std::thread input_thread(inputThread);
 
  rclcpp::WallRate loop_rate(50);
  while (rclcpp::ok()) {
    app_cmd_publisher->publish(g_app_cmd_msg);
    rclcpp::spin_some(g_node);
    loop_rate.sleep();
  }
  rclcpp::shutdown();
  
  if (input_thread.joinable()) {
      input_thread.join();
  }
  return 0;
}
