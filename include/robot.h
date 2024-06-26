#ifndef _ROBOT_H_
#define _ROBOT_H_

#include <memory>
#include <unordered_map>  
#include <functional> 
#include "rclcpp/rclcpp.hpp"
#include "robot_base.h"
#include "robot_com.h"

namespace task_control {

enum E_ROBOT_FAULT_LEVEL
{
    FAULT_LEVEL_UNKNOWN = 0,  // 未知
    FAULT_LEVEL_ERROR_1,    // 
    FAULT_LEVEL_ERROR_2,    // 
    FAULT_LEVEL_WARNN_1,    // 
    FAULT_LEVEL_WARNN_2,    // 
    FAULT_LEVEL_NORMA_1,    // 
    FAULT_LEVEL_NORMA_2,    // 
};

enum E_ROBOT_POS
{
    POS_UNKNOWN = 0,    // 未知
    POS_LEFT,           // 左边
    POS_RIGHT,          // 右边
};

enum E_ROBOT_MOVE
{
    MOVE_SPIN = 0,    // 旋转
    MOVE_ADVANCE,     // 前进
    MOVE_RECOIL,      // 后退
};

enum E_ROBOT_MODE
{
    MANUAL = 1,
    AUTO
};

enum E_ROBOT_TASK
{
    TASK_NULL = 0,    // 无任务
    TASK_MANUAL,      // 手动任务
    TASK_AUTO,        // 自动任务
};

enum E_ROBOT_STATE
{
    STATE_IDEL = 0,             // 空闲
    STATE_PAUSE,                // 暂停
    STATE_STOP,                 // 停止
    STATE_AUTO_WORKING,         // 自动作业
    STATE_MANUAL_WORKING,       // 手动作业
    STATE_FAULT,                // 故障
    STATE_EMR,                  // 急停
    STATE_EMR_RESTORE,          // 急停复位
};


enum E_FSM_STATE
{
    FSM_STATE_IDEL = 0,	        // 空闲
    FSM_STATE_READY,       	    // 准备
    FSM_STATE_WORK,        	    // 作业
    FSM_STATE_PAUSE,		    // 暂停
    FSM_STATE_CONTINUE,		    // 继续
    FSM_STATE_STOP,             // 停止
    FSM_STATE_FINISH,	        // 结束

    FSM_STATE_CHECK_MOTOR,                  // 检查电机运行状态
    FSM_STATE_FIRST_MOVE,                   // 第一次移动
    FSM_STATE_FIRST_MOVE_WAIT_RESULT,       // 第一次移动等待结果
    FSM_STATE_FIRST_RECOIL,                 // 第一次后退
    FSM_STATE_FIRST_RECOIL_WAIT_RESULT,     // 第一次后退等待结果
    FSM_STATE_FIRST_SPIN,                   // 第一次旋转
    FSM_STATE_FIRST_SPIN_WAIT_RESULT,       // 第一次旋转等待结果
    FSM_STATE_ADVANCE,                      // 一直前进
    FSM_STATE_ADVANCE_WAIT_RESULT,          // 一直前进等待结果
    FSM_STATE_ADVANCE_FIXED,                // 前进固定距离
    FSM_STATE_ADVANCE_FIXED_WAIT_RESULT,    // 前进固定距离等待结果
    FSM_STATE_RECOIL_FIXED,                 // 后退固定距离
    FSM_STATE_RECOIL_FIXED_WAIT_RESULT,     // 后退固定距离等待结果
    FSM_STATE_SPIN,                         // 旋转
    FSM_STATE_SPIN_WAIT_RESULT,             // 旋转等待结果
};

class Robot final : public RobotBase, public RobotCom {
public:
    Robot(const std::shared_ptr<rclcpp::Node>& nh);
    Robot(const Robot&) = delete;
    Robot(const Robot&&) = delete;
    Robot& operator=(const Robot&) = delete;
    Robot& operator=(const Robot&&) = delete;
    ~Robot() {}

    void init(const std::shared_ptr<rclcpp::Node>& nh) override;
    bool do_normal() override;                                                                    			
    bool start_work(int brush_v_level) override;
    bool stop_work() override;
    bool stop_move() override;
    bool emr() override;
    bool emr_restore() override;
    void mcu_to_motion_callback(const McuToMotion::SharedPtr msg) override;
    void mcu_to_task_callback(const McuToTask::SharedPtr msg) override;
    void motion_to_mcu_callback(const MotionToMcu::SharedPtr msg) override;
    void app_cmd_callback(const AppCmd::SharedPtr msg) override;
    void vision_result_callback(const VisionResult::SharedPtr msg) override;

    bool get_emer_stop_status()     { return m_mcu_to_task_msg_->emergency_stop_status; }
    bool get_suction_cup_status()   { return m_mcu_to_task_msg_->suction_cup_status; }
    bool get_motor_status()         { return m_mcu_to_task_msg_->motor_status; }
    bool get_brush_status()         { return m_mcu_to_task_msg_->brush_status; }
    bool get_drop_sign()            { return m_mcu_to_task_msg_->drop_sign; }
    int  get_exception_code()       { return m_mcu_to_task_msg_->exception_code; }
    int  get_bat_vol()              { return m_mcu_to_task_msg_->bat_vol; }

    void deal_update_task_msg();
    void deal_emer_event();
    void deal_fault_event();
    void deal_auto_work_cmd();

    bool is_fsm_running();

private:
    void set_robot_move(int task_mode, float aim_x = 0, float aim_y = 0, float aim_yaw = 0);

    TaskToMcu task_to_mcu_msg_;
    TaskToMcu task_to_mcu_msg_buf_;

    //回调数据
    McuToMotion::SharedPtr  m_mcu_to_motion_msg_;
    McuToTask::SharedPtr    m_mcu_to_task_msg_;
    MotionToMcu::SharedPtr  m_motion_to_mcu_msg_;
    VisionResult::SharedPtr m_vision_result_msg_;

	int robot_mode_ = E_ROBOT_MODE::MANUAL;
    int robot_state_ = E_ROBOT_STATE::STATE_IDEL;
    int task_ = E_ROBOT_TASK::TASK_NULL;

    bool m_isPause = false;
    bool m_isResume = false;
    bool m_isStop = false;

    //状态机相关
    int m_robot_pos;
    int m_spin_cnt;

    int m_emer_cnt;
    bool m_emer_flag = false;
    E_ROBOT_FAULT_LEVEL m_fault_level;

};

}  // namespace task_control

#endif // _ROBOT_H_
