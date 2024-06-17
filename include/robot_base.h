#ifndef _ROBOT_BASE_H_
#define _ROBOT_BASE_H_

#include "rclcpp/rclcpp.hpp"
#include "fsm.h"


namespace task_control {

class RobotBase {
public:
    RobotBase(const std::shared_ptr<rclcpp::Node>& nh) {}
    RobotBase(const RobotBase&) = delete;
    RobotBase(const RobotBase&&) = delete;
    RobotBase& operator=(const RobotBase&) = delete;
    RobotBase& operator=(const RobotBase&&) = delete;
    virtual ~RobotBase() {}
    
    virtual void init(const std::shared_ptr<rclcpp::Node>& nh) = 0;
    virtual bool do_normal() = 0;                                                                  
    virtual bool start_work() = 0;
    virtual bool stop_work() = 0;
    virtual bool stop_move() = 0;
    virtual bool emr() = 0;
    virtual bool emr_restore() = 0;
    inline bool is_work_finished() const {
        return work_is_finished_;
    }

    inline bool is_work_successed() const {
        return work_is_successed_;
    }
    inline bool spin_once() {
        return robot_fsm_.spin_once();
    }

    void cancel_task();
    FSM robot_fsm_;

private:
    bool work_is_finished_;
    bool work_is_successed_;
};

}  // namespace task_control

#endif  // _ROBOT_BASE_H_
