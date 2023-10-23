#pragma once

#include "synhal.h"

//    Copyright 2023 Christoph Hellmann Santos
//    Copyright 2014-2022 Authors of ros_canopen
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.
//

class State402
{
public:
  enum StatusWord
  {
    SW_Ready_To_Switch_On = (1 << 0),
    SW_Switched_On = (1 << 1),
    SW_Operation_enabled =(1 << 2),
    SW_Fault = (1 << 3),
    SW_Voltage_enabled = (1 << 4),
    SW_Quick_stop = (1 << 5),
    SW_Switch_on_disabled = (1 << 6),
    SW_Warning = (1 << 7),
    SW_Manufacturer_specific0 = (1 << 8),
    SW_Remote = (1 << 9),
    SW_Target_reached = (1 << 10),
    SW_Internal_limit = (1 << 11),
    SW_Operation_mode_specific0 = (1 << 12),
    SW_Operation_mode_specific1 = (1 << 13),
    SW_Manufacturer_specific1 = (1 << 14),
    SW_Manufacturer_specific2 = (1 << 15)
  };
  enum ControlWord
  {
    CW_Switch_On = 0,
    CW_Enable_Voltage = 1,
    CW_Quick_Stop = 2,
    CW_Enable_Operation = 3,
    CW_Operation_mode_specific0 = 4,
    CW_Operation_mode_specific1 = 5,
    CW_Operation_mode_specific2 = 6,
    CW_Fault_Reset = 7,
    CW_Halt = 8,
    CW_Operation_mode_specific3 = 9,
    // CW_Reserved1=10,
    CW_Manufacturer_specific0 = 11,
    CW_Manufacturer_specific1 = 12,
    CW_Manufacturer_specific2 = 13,
    CW_Manufacturer_specific3 = 14,
    CW_Manufacturer_specific4 = 15,
  };
  enum OperationMode
  {
    No_Mode = 0,
    Profiled_Position = 1,
    Velocity = 2,
    Profiled_Velocity = 3,
    Profiled_Torque = 4,
    Reserved = 5,
    Homing = 6,
    Interpolated_Position = 7,
    Cyclic_Synchronous_Position = 8,
    Cyclic_Synchronous_Velocity = 9,
    Cyclic_Synchronous_Torque = 10,
  };
  enum InternalState
  {
    Unknown = 0,
    Start = 0,
    Not_Ready_To_Switch_On = 1,
    Switch_On_Disabled = 2,
    Ready_To_Switch_On = 3,
    Switched_On = 4,
    Operation_Enable = 5,
    Quick_Stop_Active = 6,
    Fault_Reaction_Active = 7,
    Fault = 8,
  };

  State402();
  void init(uint16_t* pStatus, int8_t* pMode);

  void update(uint16_t controlword, int8_t operation_mode);

  uint16_t getStatusword() const { return *_pstatusword; };
  InternalState getState() const { return _state; };
  OperationMode getMode() const { return _mode; }
  //bool waitForNewState(uint32_t timeout, InternalState & state);
  // set the target reached bits in the state display word
  // I checked this only for profiled position mode, (mode specific bits!!)
  void set_target_reached(bool reached);
private:
  void setStatusword(InternalState is);
  void on_not_ready_to_switch_on();
  void on_switch_on_disabled();
  void on_ready_to_switch_on();
  void on_switched_on();
  void on_operation_enabled();
  void on_quickstop_active();

  void set_switch_on_disabled();
  void set_ready_to_switch_on();
  void set_switch_on();
  void set_operation_enabled();
  void set_quick_stop();

  bool is_shutdown();
  bool is_disable_voltage();
  bool is_switch_on();
  bool is_enable_operation();
  bool is_quickstop();
  bool is_faul_reset();

  void set_clr_status_bit(int bitset, int bitclr);

  void run_profiled_position_mode();
  void run_cyclic_position_mode();
  void run_interpolated_position_mode();
  void tick();

  //std::condition_variable cond_;
  //std::mutex mutex_;
  uint16_t* _pstatusword;
  int8_t* _pmode;
  //syn::CANopenNode::TPDOtrigger _tpdo_state_mode;
  uint16_t _controlword;
  InternalState _state;
  OperationMode _mode;
  OperationMode _mode_old;
  bool is_relative;
  bool is_halt;
  bool is_new_setpoint;
};

