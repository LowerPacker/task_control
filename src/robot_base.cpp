#include "robot_base.h"

namespace task_control {

void RobotBase::cancel_task() {
    if (!work_is_finished_) {
        work_is_finished_ = true;
        work_is_successed_ = false;
    }
}

}  // namespace task_control

