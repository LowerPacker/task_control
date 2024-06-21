#ifndef _FSM_H_
#define _FSM_H_

#include <functional>
#include <map>

namespace task_control {

// 状态机封装类
class FSM;
// 请始终返回false 只有当希望流程上下文结束时才返回true
typedef std::function< bool (FSM* )> FUNC_FSM;
typedef FUNC_FSM FUNC_FSM_ENTER;
typedef FUNC_FSM FUNC_FSM_RUN;
typedef FUNC_FSM FUNC_FSM_EXIT;
typedef struct {
    int state_no;                   //状态机 状态序号
    FUNC_FSM_ENTER func_enter;      //状态机 进入状态函数
    FUNC_FSM_RUN func_run;          //状态机 状态运行函数
    FUNC_FSM_EXIT func_exit;        //状态机 状态退出函数
}FSM_STATE;

class FSM
{
public:
    FSM();
    FSM(const FSM&) = delete;
    FSM(const FSM&&) = delete;
    FSM& operator=(const FSM&) = delete;
    FSM& operator=(const FSM&&) = delete;
    ~FSM();

    bool init(int state_no = 0);
    bool add_state(int state_no, FUNC_FSM_ENTER func_enter,FUNC_FSM_RUN func_run, FUNC_FSM_EXIT func_exit);
    bool add_state(int state_no, FUNC_FSM_RUN func_run);
    bool goto_state(int state_no);
    bool spin_once();
    bool restart();
    void reset_start_sec_in_state();
    time_t get_sec_in_state();
    time_t get_sec_in_fsm();
    inline void pause() {
        m_sotp_running = true;
    }

    inline int get_state() {
        return m_current_state;
    }

    inline int get_last_state() {
        return m_last_state;
    }
    inline int get_last_run_state() {
        return m_last_run_state;
    }
    bool is_finished = false;

private:
    bool m_sotp_running;
    int m_current_state;
    int m_last_state;
    int m_last_run_state;
    time_t m_start_sec_in_state;
    time_t m_start_sec_in_fsm;
    FUNC_FSM_RUN m_run_func;
    std::map<int, FSM_STATE*> m_allstates;
};

}  // namespace task_control

#endif // _FSM_H_
