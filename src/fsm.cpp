#include "fsm.h"

namespace task_control {

FSM::FSM() : m_sotp_running(false), m_current_state(0), m_last_state(0) {
}

FSM::~FSM() {
    for(auto& kv : m_allstates) {
        delete kv.second;
    }
}

bool FSM::init(int state_no) {
    // 列表不存在的状态
    if(m_allstates.find(state_no) == m_allstates.end()) {
        return false;
    }

    m_sotp_running = false;
    m_start_sec_in_fsm = m_start_sec_in_state = time(0);
    m_current_state = state_no;
    m_run_func = m_allstates.at(m_current_state)->func_run;
    return true;
}

bool FSM::add_state(int state_no, FUNC_FSM_ENTER func_enter, FUNC_FSM_RUN func_run, FUNC_FSM_EXIT func_exit) {
    if(m_allstates.find(state_no) != m_allstates.end()) {
        return false;
    }

    auto state = new FSM_STATE();
    state->state_no = state_no;
    state->func_enter = func_enter;
    state->func_run = func_run;
    state->func_exit = func_exit;
    m_allstates.insert(std::pair<int, FSM_STATE*>(state_no, state));

    return true;
}

bool FSM::add_state(int state_no, FUNC_FSM_RUN func_run) {
    return add_state(state_no, NULL, func_run, NULL);
}

bool FSM::goto_state(int state_no) {
    m_last_run_state = m_current_state;
    if(m_current_state == state_no) {
        return true;
    }

    if(m_allstates.find(state_no) == m_allstates.end()) {
        return false;
    }

    auto old_state = m_allstates.at(m_current_state);
    auto new_state = m_allstates.at(state_no);
    // 执行老状态的结束
    if (old_state->func_exit) {
        (old_state->func_exit)(this);
    }
    // 执行新状态的开始
    if(new_state->func_enter) {
        (new_state->func_enter)(this);
    }

    reset_start_sec_in_state();
    m_last_state = m_current_state;
    m_current_state = state_no;
    m_run_func = new_state->func_run;
    return true;
}

bool FSM::spin_once() {
    if (m_sotp_running) {
        return true;
    }

    return (m_run_func)(this);
}

void FSM::reset_start_sec_in_state() {
    m_start_sec_in_state = time(0);
}

time_t FSM::get_sec_in_state() {
    return time(0) - m_start_sec_in_state;
}

time_t FSM::get_sec_in_fsm() {
    return time(0) - m_start_sec_in_fsm;
}

bool FSM::restart() {
    if(m_sotp_running) {
        m_sotp_running = false;
        m_start_sec_in_state = time(0);
    }
    return true;
}

}  // namespace task_control


