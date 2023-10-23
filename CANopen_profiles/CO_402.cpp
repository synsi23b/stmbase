#include "CO_402.h"
#include "synhal.h"
#include "OD.h"
#include <stdio.h>

State402::State402()
{
}

void State402::init(uint16_t* pStatus, int8_t* pMode)
{
    _pstatusword = pStatus;
    _pmode = pMode;
    _controlword = 0;
    _state = Not_Ready_To_Switch_On;
    _mode = No_Mode;
    _mode_old = No_Mode;
    // get flags for status + mode-display
    //_flagsPDO_stat_mode = syn::CANopenNode::getFlagsPDO(0x1801);
    //_tpdo_state_mode.init(OD_ENTRY_H6061_modesOfOperationDisplay);
}

void State402::update(uint16_t controlword, int8_t mode)
{
    if (controlword != _controlword)
    {
        printf("new ctrle %x -> %x\n", _controlword, controlword);
        _controlword = controlword;
        switch (_state)
        {
        case InternalState::Not_Ready_To_Switch_On:
            on_not_ready_to_switch_on();
            break;
        case InternalState::Switch_On_Disabled:
            on_switch_on_disabled();
            break;
        case InternalState::Ready_To_Switch_On:
            on_ready_to_switch_on();
            break;
        case InternalState::Switched_On:
            on_switched_on();
            break;
        case InternalState::Operation_Enable:
            on_operation_enabled();
            break;
        case InternalState::Quick_Stop_Active:
            on_quickstop_active();
            break;
        case InternalState::Fault_Reaction_Active:
            break;
        case InternalState::Fault:
            printf("Fault\n");
            break;
        default:
            break;
        }
        // syn::CANopenNode::requestTPDO(_flagsPDO_stat_mode, 0);
        //_tpdo_state_mode.trigger();
    }
    if (mode != *_pmode)
    {
        printf("new mode %x -> %x\n", *_pmode, mode);
        *_pmode = mode;
        _mode = static_cast<OperationMode>(mode);
        // syn::CANopenNode::requestTPDO(_flagsPDO_stat_mode, 0);
        //_tpdo_state_mode.trigger();
    }
    //tick();
}

void State402::set_clr_status_bit(int bitset, int bitclr)
{
    syn::Atomic a;
    uint16_t state = *_pstatusword;
    //uint16_t state2 = state;
    state |= bitset;
    state &= ~bitclr;
    *_pstatusword = state;
    //printf("new state %x -> %x\n", state2, state);
}

void State402::on_not_ready_to_switch_on() { set_switch_on_disabled(); }

void State402::on_switch_on_disabled()
{
    if (is_shutdown())
    {
        set_ready_to_switch_on();
    }
}

void State402::on_ready_to_switch_on()
{
    if (is_disable_voltage())
    {
        set_switch_on_disabled();
    }
    if (is_switch_on())
    {
        set_switch_on();
    }
    if (is_faul_reset())
    {
        set_ready_to_switch_on();
    }
}

void State402::on_switched_on()
{
    if (is_disable_voltage())
    {
        set_switch_on_disabled();
    }
    if (is_shutdown())
    {
        set_ready_to_switch_on();
    }
    if (is_enable_operation())
    {
        set_operation_enabled();
    }
}

void State402::on_operation_enabled()
{
    if (is_disable_voltage())
    {
        set_switch_on_disabled();
    }
    if (is_shutdown())
    {
        set_ready_to_switch_on();
    }
    if (is_switch_on())
    {
        set_switch_on();
    }
    if (is_quickstop())
    {
        set_quick_stop();
    }
    {
        // std::scoped_lock<std::mutex> lock(w_mutex);
        is_relative = (((_controlword >> 6) & 1U) == 1U);
        is_halt = (((_controlword >> 8) & 1U) == 1U);
        is_new_setpoint = (((_controlword >> 4) & 1U) == 1U);
    }

    if (_mode_old != _mode)
    {
        printf("run mode switch");
        // run mode changed, stop previous executer
        //if (_runner != NULL)
        //{
        //    switch (_mode_old)
        //    {
        //    case Cyclic_Synchronous_Position:
        //        // start_sync_pos_mode();
        //        printf("Joined cyclic_position_mode thread.\n");
        //        break;
        //    case Profiled_Position:
        //        // start_profile_pos_mode();
        //        printf("Joined profiled_position_mode thread.\n");
        //        break;
        //    case Interpolated_Position:
        //        printf("Joined interpolated_position_mode thread.\n");
        //        break;
        //    default:
        //        break;
        //    }
        //    _runner = NULL;
        //}

        // store the new mode as the current mode
        _mode_old = _mode;
        // and start the new running mode, if any
        //switch (_mode)
        //{
        //case Cyclic_Synchronous_Position:
        //    // start_sync_pos_mode();
        //    _runner = &State402::run_cyclic_position_mode;
        //    break;
        //case Profiled_Position:
        //    // start_profile_pos_mode();
        //    _runner = &State402::run_profiled_position_mode;
        //    break;
        //case Interpolated_Position:
        //    // start_interpolated_pos_mode();
        //    _runner = &State402::run_interpolated_position_mode;
        //    break;
        //default:
        //    break;
        //}
    }
}

void State402::on_quickstop_active()
{
    if (is_enable_operation())
    {
        set_operation_enabled();
    }
    if (is_disable_voltage())
    {
        set_switch_on_disabled();
    }
}

void State402::set_switch_on_disabled()
{
    printf("Switch_On_Disabled\n");
    _state = InternalState::Switch_On_Disabled;
    set_clr_status_bit(SW_Switch_on_disabled,
                       SW_Ready_To_Switch_On | SW_Switched_On | SW_Operation_enabled | SW_Fault);
}

void State402::set_ready_to_switch_on()
{
    printf("Ready_To_Switch_On\n");
    _state = InternalState::Ready_To_Switch_On;
    set_clr_status_bit(SW_Ready_To_Switch_On | SW_Quick_stop,
                       SW_Switched_On | SW_Operation_enabled | SW_Fault | SW_Switch_on_disabled);
}

void State402::set_switch_on()
{
    printf("Switched_On\n");
    _state = InternalState::Switched_On;
    set_clr_status_bit(SW_Ready_To_Switch_On | SW_Switched_On | SW_Quick_stop,
                       SW_Operation_enabled | SW_Fault | SW_Switch_on_disabled);
}

void State402::set_operation_enabled()
{
    printf("Operation_Enable\n");
    _state = InternalState::Operation_Enable;
    set_clr_status_bit(SW_Ready_To_Switch_On | SW_Switched_On | SW_Operation_enabled | SW_Quick_stop,
                       SW_Switch_on_disabled | SW_Fault);
}

void State402::set_quick_stop()
{
    printf("Quick_Stop_Active\n");
    _state = InternalState::Quick_Stop_Active;
    set_clr_status_bit(SW_Ready_To_Switch_On | SW_Switched_On | SW_Operation_enabled,
                       SW_Quick_stop | SW_Switch_on_disabled | SW_Fault);
}

bool State402::is_shutdown()
{
    // std::scoped_lock<std::mutex> lock(w_mutex);
    bool fr_unset = ((_controlword >> CW_Fault_Reset) & 1U) == 0U;
    bool qs_set = ((_controlword >> CW_Quick_Stop) & 1U) == 1U;
    bool ev_set = ((_controlword >> CW_Enable_Voltage) & 1U) == 1U;
    bool so_unset = ((_controlword >> CW_Switch_On) & 1U) == 0U;

    if (fr_unset && qs_set && ev_set && so_unset)
    {
        printf("Received Shutdown.\n");
        return true;
    }
    return false;
}

bool State402::is_disable_voltage()
{
    // std::scoped_lock<std::mutex> lock(w_mutex);
    bool fr_unset = ((_controlword >> CW_Fault_Reset) & 1U) == 0U;
    bool ev_unset = ((_controlword >> CW_Enable_Voltage) & 1U) == 0U;

    if (fr_unset && ev_unset)
    {
        printf("Received Disable Voltage.\n");
        return true;
    }
    return false;
}

bool State402::is_switch_on()
{
    // std::scoped_lock<std::mutex> lock(w_mutex);
    bool fr_unset = ((_controlword >> CW_Fault_Reset) & 1U) == 0U;
    bool eo_unset = ((_controlword >> CW_Enable_Operation) & 1U) == 0U;
    bool qs_set = ((_controlword >> CW_Quick_Stop) & 1U) == 1U;
    bool ev_set = ((_controlword >> CW_Enable_Voltage) & 1U) == 1U;
    bool so_set = ((_controlword >> CW_Switch_On) & 1U) == 1U;
    if (fr_unset && eo_unset && qs_set && ev_set && so_set)
    {
        printf("Received Switch On.\n");
        return true;
    }
    return false;
}

bool State402::is_enable_operation()
{
    // std::scoped_lock<std::mutex> lock(w_mutex);
    bool fr_unset = ((_controlword >> CW_Fault_Reset) & 1U) == 0U;
    bool eo_set = ((_controlword >> CW_Enable_Operation) & 1U) == 1U;
    bool qs_set = ((_controlword >> CW_Quick_Stop) & 1U) == 1U;
    bool ev_set = ((_controlword >> CW_Enable_Voltage) & 1U) == 1U;
    bool so_set = ((_controlword >> CW_Switch_On) & 1U) == 1U;
    if (fr_unset && eo_set && qs_set && ev_set && so_set)
    {
        printf("Received Enable Operation.\n");
        return true;
    }
    return false;
}

bool State402::is_quickstop()
{
    // std::scoped_lock<std::mutex> lock(w_mutex);
    bool fr_unset = ((_controlword >> CW_Fault_Reset) & 1U) == 0U;
    bool qs_unset = ((_controlword >> CW_Quick_Stop) & 1U) == 0U;
    bool ev_set = ((_controlword >> CW_Enable_Voltage) & 1U) == 1U;
    if (fr_unset && qs_unset && ev_set)
    {
        printf("Received Quick Stop.\n");
        return true;
    }
    return false;
}

bool State402::is_faul_reset()
{
    // std::scoped_lock<std::mutex> lock(w_mutex);
    bool fr_set = ((_controlword >> CW_Fault_Reset) & 1U) == 1U;
    if (fr_set)
    {
        printf("Received Fault Reset.\n");
        return true;
    }
    return false;
}

void State402::set_target_reached(bool reached)
{
    uint16_t reached_bit = *_pstatusword & SW_Target_reached;
    if(reached && reached_bit == 0)
    {
        set_clr_status_bit(SW_Target_reached, SW_Operation_mode_specific0);
    }
    else if(!reached && reached_bit != 0)
    {
        set_clr_status_bit(SW_Operation_mode_specific0, SW_Target_reached);
    }
}

// void State402::run_profiled_position_mode()
// {
//     static bool once = false;
//     int32_t target = OD_RAM.x607A_targetPosition;
//     int32_t actual = OD_RAM.x6064_positionActualValue;
//     if (actual < target)
//     {
//         once = true;
//         OD_RAM.x6064_positionActualValue = actual + 1;
//     }
//     else if (actual > target)
//     {
//         once = true;
//         OD_RAM.x6064_positionActualValue = actual - 1;
//     }
//     else if (once)
//     {
//         once = false;
//         set_clr_status_bit(SW_Target_reached, SW_Operation_mode_specific0);
//     }
// }

// void State402::run_cyclic_position_mode()
// {
//     run_profiled_position_mode();
// }

// void State402::run_interpolated_position_mode()
// {
//     run_profiled_position_mode();
// }

// void State402::tick()
// {
//     static syn::DeadlineTimer dltimer(2); // run at 2 millisec interval timeout

//     if (_runner != NULL)
//     {
//         if (dltimer.is_expired())
//         {
//             dltimer.reset(2);
//             (this->*_runner)();
//         }
//     }
// }

void State402::setStatusword(InternalState is)
{
    static const uint16_t r = SW_Ready_To_Switch_On;
    static const uint16_t s = SW_Switched_On;
    static const uint16_t o = SW_Operation_enabled;
    static const uint16_t f = SW_Fault;
    static const uint16_t q = SW_Quick_stop;
    static const uint16_t d = SW_Switch_on_disabled;

    if (is == _state)
    {
        return;
    }
    uint16_t *_pstatusword = 0;

    switch (is)
    {
    case Not_Ready_To_Switch_On:
        *_pstatusword = q;
        break;
    case Switch_On_Disabled:
        *_pstatusword = (d | q | 0 | 0 | 0 | 0);
        break;
    case Ready_To_Switch_On:
        *_pstatusword = (0 | q | 0 | 0 | 0 | r);
        break;
    case Switched_On:
        *_pstatusword = (0 | q | 0 | 0 | s | r);
        break;
    case Operation_Enable:
        *_pstatusword = (0 | q | 0 | o | s | r);
        break;
    case Quick_Stop_Active:
        *_pstatusword = (0 | 0 | 0 | o | s | r);
        break;
    case Fault_Reaction_Active:
        *_pstatusword = (0 | q | f | o | s | r);
        break;
    case Fault:
        *_pstatusword = (0 | q | f | 0 | 0 | 0);
        break;
    default:
        /// @todo Throw error here.
        break;
    }
}