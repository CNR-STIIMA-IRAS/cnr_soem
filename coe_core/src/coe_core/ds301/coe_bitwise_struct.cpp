#include <coe_core/coe_string_utilities.h>
#include <coe_core/coe_utilities.h>
#include <coe_core/ds301/coe_bitwise_struct.h>

namespace coe_core
{
  namespace ds301
  {

    std::string to_string(const ds301::error_register_t &in)
    {
      std::string ret;
      uint8_t tmp;
      std::memcpy(&tmp, &in, sizeof(uint8_t));
      ret += " Error Register: " + std::to_string(tmp) + " == " + coe_core::to_string_hex<uint8_t>(tmp) +
             " == " + coe_core::to_string_bin(tmp) + std::string("\n");
      ret += " " + (in.bits.generic_error ? std::string("1") : std::string("0")) + " .generic_error           " +
             std::string("\n");
      ret += " " + (in.bits.current ? std::string("1") : std::string("0")) + " .current                 " +
             std::string("\n");
      ret += " " + (in.bits.voltage ? std::string("1") : std::string("0")) + " .volt                    " +
             std::string("\n");
      ret += " " + (in.bits.temperature ? std::string("1") : std::string("0")) + " .temp.                   " +
             std::string("\n");
      ret += " " + (in.bits.communication_error ? std::string("1") : std::string("0")) + " .com. error              " +
             std::string("\n");
      ret += " " + (in.bits.device_profile_specific ? std::string("1") : std::string("0")) +
             " .device_profile_specific " + std::string("\n");
      ret += " " + (in.bits.reserved ? std::string("1") : std::string("0")) + " .reserved                " +
             std::string("\n");
      ret += " " + (in.bits.manufacturer_specific ? std::string("1") : std::string("0")) +
             " .manufacturer            " + std::string("\n");
      return ret;
    }

    std::string to_string(const manufacturer_status_register_t &in)
    {
      std::string ret;
      uint32_t tmp;
      std::memcpy(&tmp, &in, sizeof(uint32_t));
      ret += " Manufacturer staus register: " + coe_core::to_string_hex(tmp) + std::string(", ") +
             coe_core::to_string_bin(in) + std::string("\n");
      ret += " " + (in.bits.problem ? std::string("1") : std::string("0")) + " .problem                  \n";
      ret +=
          " " + (in.bits.problem_depenent ? std::string("1") : std::string("0")) + " .problem_depenent: " +
          (in.bits.problem_depenent == 0x1
               ? std::string("Under voltage: The power supply is shut off or it has too high an impedance.")
           : in.bits.problem_depenent == 0x2
               ? std::string("Over voltage: The power supply voltage is too large, or the servo drive did not succeed "
                             "in absorbing the kinetic energy while braking a load. A shunt resistor may be needed.")
           : in.bits.problem_depenent == 0x3
               ? std::string("Error not expalined in documentation.. Under and over voltage?????")
           : in.bits.problem_depenent == 0x4 ? std::string("Error not expalined in documentation")
           : in.bits.problem_depenent == 0x5 ? std::string("Short circuit: The motor or its wiring may be defective..")
           : in.bits.problem_depenent == 0x6 ? std::string("Temperature: The drive is overheating.")
           : in.bits.problem_depenent == 0x7 ? std::string("Error not expalined in documentation")
           : in.bits.problem_depenent == 0x8 ? std::string("Error not expalined in documentation")
                                             : "Look at the complete bit map");
      ret += "  \n";
      ret += " " + (in.bits.motor_on ? std::string("1") : std::string("0")) + "  .motor_on                 \n";
      ret += " " + (in.bits.reference_mode ? std::string("1") : std::string("0")) + "  .reference_mode           \n";
      ret += " " + (in.bits.motor_failure_latched ? std::string("1") : std::string("0")) +
             "  .motor_failure_latched    \n";
      ret += " " + coe_core::to_string_hex(in.bits.unit_mode) + "  .unit_mode                \n";
      ret +=
          " " + (in.bits.gain_scheduling_on ? std::string("1") : std::string("0")) + "  .gain_scheduling_on       \n";
      ret += " " + (in.bits.homing_being_processed ? std::string("1") : std::string("0")) +
             "  .homing_being_processed   \n";
      ret += " " + (in.bits.program_running ? std::string("1") : std::string("0")) + "  .program_running          \n";
      ret += " " + (in.bits.current_limit_on ? std::string("1") : std::string("0")) + "  .current_limit_on         \n";
      ret += " " + (in.bits.motion_status_reflection ? std::string("1") : std::string("0")) +
             "  .motion_status_reflection : " +
             std::string(
                 in.bits.unit_mode == 2
                     ? (in.bits.motion_status_reflection == 0x0 ? "Not applicable for this mode."
                        : in.bits.motion_status_reflection == 0x1
                            ? "The reference to the speed controller equals the speed target. (This is always the case "
                              "if no profiler mode is used: PM=0.), OR The motor is off (MO=0)."
                        : in.bits.motion_status_reflection == 0x2
                            ? "The software reference to the speed controller differs from the speed target. In "
                              "software profiled reference mode (PM=1, RM=0), this is the case while accelerating, "
                              "decelerating or smoothing to target speed JV."
                            : "Reserved.")
                     : (in.bits.motion_status_reflection == 0x0
                            ? "Motor position stabilized. The feedback position is steady within the ranges defined by "
                              "TR. Note that value 0 is applicable only when there is a defined position target, as in "
                              "PTP."
                        : in.bits.motion_status_reflection == 0x1
                            ? "The reference for the position controller is stationary, or the motor is off (MO=0)."
                        : in.bits.motion_status_reflection == 0x2
                            ? "The reference to the position controller is dynamically controlled by one of the "
                              "optional motion profilers: PTP, Jog, PT or PVT."
                            : "Reserved."));
      ret += "  \n";
      ret +=
          " " + (in.bits.digital_hall_sensors ? std::string("1") : std::string("0")) + " .digital_hall_sensors     \n";
      ret += " " + (in.bits.CPU_status_not_ok ? std::string("1") : std::string("0")) + " .CPU_status_not_ok        \n";
      ret += " " + (in.bits.stopped_by_a_limit ? std::string("1") : std::string("0")) + " .stopped_by_a_limit       \n";
      ret +=
          " " + (in.bits.error_in_user_program ? std::string("1") : std::string("0")) + " .error_in_user_program    \n";

      return ret;
    }

    std::string to_string(const emergency_events_t &in)
    {
      std::string ret;
      uint16_t tmp;
      std::memcpy(&tmp, &in, sizeof(uint16_t));
      ret += " Emergency Events: 0x" + coe_core::to_string_hex(tmp) + ", " + coe_core::to_string_bin(in) +
             std::string("\n");
      ret += std::string(" ") + (in.bits.CAN_message_lost ? "1" : "0") +
             std::string(" .CAN_message_lost                         \n");
      ret += std::string(" ") + (in.bits.protocol_error ? "1" : "0") +
             std::string(" .protocol_error                           \n");
      ret += std::string(" ") + (in.bits.attempt_to_access_an_unconfigured_rpdo ? "1" : "0") +
             std::string(" .attempt_to_access_an_unconfigured_rpdo   \n");
      ret += std::string(" ") + (in.bits.heartbeat_event ? "1" : "0") +
             std::string(" .heartbeat_event                          \n");
      ret += std::string(" ") + (in.bits.fatal_cpu_error ? "1" : "0") +
             std::string(" .fatal_cpu_error                          \n");
      ret += std::string(" ") + (in.bits.user_program_aborted_by_an_error ? "1" : "0") +
             std::string(" .user_program_aborted_by_an_error         \n");
      ret += std::string(" ") + (in.bits.request_by_user_program__emit_function ? "1" : "0") +
             std::string(" .request_by_user_program__emit_function   \n");
      ret += std::string(" ") + (in.bits.motor_shut_down_by_fault ? "1" : "0") +
             std::string(" .motor_shut_down_by_fault                 \n");
      ret += std::string(" ") + (in.bits.rpdo_returned_an_error ? "1" : "0") +
             std::string(" .rpdo_returned_an_error                   \n");
      ret += std::string(" ") + (in.bits.ds402_ip_underflow ? "1" : "0") +
             std::string(" .ds402_ip_underflow                       \n");
      return ret;
    }

    std::string to_string(const predefined_error_field_t &in)
    {
      std::string ret;
      ret += " Predefined error field: " + coe_core::to_string_hex<uint32_t>((uint32_t &)in) + ", " +
             coe_core::to_string_bin<uint32_t>((uint32_t &)in) + std::string("\n");
      ret += " " + coe_core::to_string_hex<uint16_t>(in.bits.error_code) + ", " +
             coe_core::to_string_bin<uint16_t>(in.bits.error_code) + " .Error code          \n";
      ret += " " + coe_core::to_string_hex<uint8_t>(in.bits.error_register) + ", " +
             coe_core::to_string_bin<uint16_t>(in.bits.error_register) + " .Error Register      \n";
      ret += " " + coe_core::to_string_hex<uint8_t>(in.bits.manufacturer_specific_error_code) + ", " +
             coe_core::to_string_bin<uint16_t>(in.bits.manufacturer_specific_error_code) + " .Manufacturer Error  \n";
      return ret;
    }

    uint8_t &operator<<(uint8_t &lhs, const error_register_t &rhs)
    {
      lhs = rhs.value;
      return lhs;
    }

    error_register_t &operator<<(error_register_t &lhs, const uint8_t &rhs)
    {
      lhs.value = rhs;
      return lhs;
    }

    uint16_t &operator<<(uint16_t &lhs, const emergency_events_t &rhs)
    {
      lhs = rhs.value;
      return lhs;
    }
    emergency_events_t &operator<<(emergency_events_t &lhs, const uint16_t &rhs)
    {
      lhs.value = rhs;
      return lhs;
    }
    uint32_t &operator<<(uint32_t &lhs, const predefined_error_field_t &rhs)
    {
      lhs = rhs.value;
      return lhs;
    }
    predefined_error_field_t &operator<<(predefined_error_field_t &lhs, const uint32_t &rhs)
    {
      lhs.value = rhs;
      return lhs;
    }
    uint32_t &operator<<(uint32_t &lhs, const manufacturer_status_register_t &rhs)
    {
      lhs = rhs.value;
      return lhs;
    }
    manufacturer_status_register_t &operator<<(manufacturer_status_register_t &lhs, const uint32_t &rhs)
    {
      lhs.value = rhs;
      return lhs;
    }

    std::ostream &operator<<(std::ostream &str, const error_register_t &in)
    {
      str << to_string(in);
      return str;
    }
    std::ostream &operator<<(std::ostream &str, const manufacturer_status_register_t &in)
    {
      str << to_string(in);
      return str;
    }
    std::ostream &operator<<(std::ostream &str, const emergency_events_t &in)
    {
      str << to_string(in);
      return str;
    }
    std::ostream &operator<<(std::ostream &str, const predefined_error_field_t &in)
    {
      str << to_string(in);
      return str;
    }

  }
}
