#ifndef ESA_EWDL_ETHERCAT_COMMON_H
#define ESA_EWDL_ETHERCAT_COMMON_H
// STL
#include <string>
#include <map>
// soem
#include "ethercat.h"


namespace esa { namespace ewdl { namespace ethercat {

/* Mode of Operation */
enum mode_of_operation_t : int8
{
  Q_PROGRAM = -1,                    // Q Program Mode (ESA specific)
  PROFILE_POSITION = 1,              // Profile Position Mode
  PROFILE_VELOCITY = 3,              // Profile Velocity Mode
  TORQUE_PROFILE = 4,                // Torque Profile Mode
  HOMING = 6,                        // Homing Mode
  CYCLIC_SYNCHRONOUS_POSITION = 8,   // Cyclic Synchronous Position Mode
  CYCLIC_SYNCHRONOUS_VELOCITY = 9,   // Cyclic Synchronous Velocity Mode
};
// In SM synchronization mode, PP, PV, TQ, HM and Q modes are supported.
// In DC synchronization mode, CSP, CSV and HM modes are supported.


/* Error Codes */
static std::map<uint16, std::string> error_codes
{
  { 0x7500, "EtherCAT communication error"},
  { 0xFF01, "Over Current"},
  { 0xFF02, "Over Voltage"},
  { 0xFF03, "Over Temperature"},
  { 0xFF04, "Open Motor Winding"},
  { 0xFF05, "Internal Voltage Bad"},
  { 0xFF06, "Position Limit"},
  { 0xFF07, "Bad Encoder"},
  { 0xFF08, "reserved"},
  { 0xFF09, "reserved"},
  { 0xFF0A, "Excess regen"},
  { 0xFF0B, "Safe Torque Off"},
  { 0xFF31, "CW Limit"},
  { 0xFF32, "CCW Limit"},
  { 0xFF33, "CW Limit and CCW Limit"},
  { 0xFF34, "Current Foldback"},
  { 0xFF35, "Move while Disabled"},
  { 0xFF36, "Under Voltage"},
  { 0xFF37, "Blank Q segment"},
  { 0xFF41, "Save Failed"},
  { 0xFFFF, "Other Error"},
};


/* Alarm Codes */
static std::map<uint32, std::string> alarm_codes
{
  { 0x00000001, "Position Limit"},
  { 0x00000002, "CCW Limit"},
  { 0x00000004, "CW Limit"},
  { 0x00000008, "Over Temperature"},
  { 0x00000010, "Internal Voltage Bad"},
  { 0x00000020, "Over Voltage"},
  { 0x00000040, "Under Voltage"},
  { 0x00000080, "Over Current"},
  { 0x00000100, "Open Motor Winding"},
  { 0x00000200, "Bad Encoder"},
  { 0x00000400, "Communication Error"},
  { 0x00000800, "Save Failed"},
  { 0x00001000, "No Move"},
  { 0x00002000, "Current Foldback"},
  { 0x00004000, "Blank Q Segment"},
  { 0x00008000, "NV Memory Double Error"},
  { 0x00010000, "Excess Regen"},
  { 0x00020000, "-"},
  { 0x00040000, "Safe Torque Off"},
  { 0x00080000, "-"},
  { 0x00100000, "-"},
};


template<class T>
int writeSDO(const uint16 slave, const uint16 index, const uint8 sub_index, const T value)
{
  int wkc = 0;

  T data = value; int size_of_data = sizeof(data);
  wkc += ec_SDOwrite(slave, index, sub_index, FALSE, size_of_data, &data, EC_TIMEOUTRXM);

  return wkc;
}


template<class T>
int writeSDO(const uint16 slave, const uint16 index, const uint8 sub_index, T *value)
{
  int wkc = 0;

  T *data = value; int size_of_data = sizeof(data);
  wkc += ec_SDOwrite(slave, index, sub_index, TRUE, size_of_data, data, EC_TIMEOUTRXM);

  return wkc;
}


template<class T>
int readSDO(const uint16 slave, const uint16 index, const uint8 sub_index, T &value)
{
  int wkc = 0;

  T data = value; int size_of_data = sizeof(data);
  wkc += ec_SDOread(slave, index, sub_index, FALSE, &size_of_data, &data, EC_TIMEOUTRXM);

  value = data;

  return wkc;
}


template<class T>
int readSDO(const uint16 slave, const uint16 index, const uint8 sub_index, T *value)
{
  int wkc = 0;

  T *data = value; int size_of_data = sizeof(data);
  wkc += ec_SDOread(slave, index, sub_index, TRUE, &size_of_data, data, EC_TIMEOUTRXM);

  *value = *data;

  return wkc;
}


inline void print_ec_state(uint16 slave_idx)
{
  switch (ec_slave[slave_idx].state)
  {
    case EC_STATE_NONE:
      printf("%s: EC_STATE: %s\n", ec_slave[slave_idx].name, "NONE");
      break;
    case EC_STATE_INIT:
      printf("%s: EC_STATE: %s\n", ec_slave[slave_idx].name, "INIT");
      break;
    case EC_STATE_PRE_OP:
      printf("%s: EC_STATE: %s\n", ec_slave[slave_idx].name, "PRE_OP");
      break;
    case EC_STATE_BOOT:
      printf("%s: EC_STATE: %s\n", ec_slave[slave_idx].name, "BOOT");
      break;
    case EC_STATE_SAFE_OP:
      printf("%s: EC_STATE: %s\n", ec_slave[slave_idx].name, "SAFE_OP");
      break;
    case EC_STATE_OPERATIONAL:
      printf("%s: EC_STATE: %s\n", ec_slave[slave_idx].name, "OPERATIONAL");
      break;
    //case EC_STATE_ACK:
    //  ROS_INFO("%s: ESM: %s", ec_slave[slave].name, "EC_STATE_ACK");
    //  break;
    case EC_STATE_PRE_OP + EC_STATE_ERROR:
      printf("%s: EC_STATE: %s + %s\n", ec_slave[slave_idx].name, "PRE_OP", "ERROR");
      break;
    case EC_STATE_SAFE_OP + EC_STATE_ERROR:
      printf("%s: EC_STATE: %s + %s\n", ec_slave[slave_idx].name, "SAFE_OP", "ERROR");
      break;
    case EC_STATE_OPERATIONAL + EC_STATE_ERROR:
      printf("%s: EC_STATE: %s + %s\n", ec_slave[slave_idx].name, "OPERATIONAL", "ERROR");
      break;
  }
}


inline void print_sm(uint16 slave, int sm)
{
  uint16 A = ec_slave[slave].SM[sm].StartAddr;
  uint16 L = ec_slave[slave].SM[sm].SMlength;
  uint32 F = ec_slave[slave].SM[sm].SMflags;
  uint8 Type = ec_slave[slave].SMtype[sm];

  printf("SM%d A:%4.4x L:%4d F:%8.8x Type:%d\n", sm, A, L, F, Type);
}


inline void print_fmmu(uint16 slave, int fmmu)
{
  uint32 Ls = ec_slave[slave].FMMU[fmmu].LogStart;
  uint16 Ll = ec_slave[slave].FMMU[fmmu].LogLength;
  uint8 Lsb = ec_slave[slave].FMMU[fmmu].LogStartbit;
  uint8 Leb = ec_slave[slave].FMMU[fmmu].LogEndbit;
  uint16 Ps = ec_slave[slave].FMMU[fmmu].PhysStart;
  uint8 Psb = ec_slave[slave].FMMU[fmmu].PhysStartBit;
  uint8 Ty = ec_slave[slave].FMMU[fmmu].FMMUtype;
  uint8 Act = ec_slave[slave].FMMU[fmmu].FMMUactive;

  printf("FMMU%d Ls:%.8x Ll:%4.2d Lsb:%d Leb:%d Ps:%.4x Psb:%d Ty:%.2d Act:%.2d\n", fmmu, Ls, Ll, Lsb, Leb, Ps, Psb, Ty, Act);
}


// inline void print_slave_status(const uint16 slave_idx)
// {
//   if ((tx_pdo[slave_idx].status_word >> 0) & 0x01)   // Ready to Switch On
//   {
//     printf("Slave[%d]: Ready to Switch On\n", slave_idx);
//   }
//   if ((tx_pdo[slave_idx].status_word >> 1) & 0x01)   // Switched On
//   {
//     printf("Slave[%d]: Switched On\n", slave_idx);
//   }
//   if ((tx_pdo[slave_idx].status_word >> 2) & 0x01)   // Operation Enabled
//   {
//     printf("Slave[%d]: Operation Enabled\n", slave_idx);
//   }
//   if ((tx_pdo[slave_idx].status_word >> 3) & 0x01)   // Fault
//   {
//     printf("Slave[%d]: Fault!!\n", slave_idx);
//   }
//   if ((tx_pdo[slave_idx].status_word >> 4) & 0x01)   // Voltage Enabled
//   {
//     printf("Slave[%d]: Voltage Enabled\n", slave_idx);
//   }
//   if ((tx_pdo[slave_idx].status_word >> 5) & 0x01)   // Quick Stop
//   {
//     printf("Slave[%d]: Quick Stop Enabled\n", slave_idx);
//   }
//   if ((tx_pdo[slave_idx].status_word >> 6) & 0x01)   // Switch On Disabled
//   {
//     printf("Slave[%d]: Switch On Disabled\n", slave_idx);
//   }
//   if ((tx_pdo[slave_idx].status_word >> 7) & 0x01)   // Warning
//   {
//     printf("Slave[%d]: Warning\n", slave_idx);
//   }
//
//   switch (tx_pdo[slave_idx].mode_of_operation_display)
//   {
//     case 6:   // HOMING
//       if ((tx_pdo[slave_idx].status_word >> 10) & 0x01)           // Target Reached
//       {
//         printf("Slave[%d]: Target Reached.\n", slave_idx);
//       }
//       if ((tx_pdo[slave_idx].status_word >> 11) & 0x01)           // Internal Limit Active
//       {
//         printf("Slave[%d]: Internal Limit Active\n", slave_idx);
//       }
//       if ((tx_pdo[slave_idx].status_word >> 12) & 0x01)           // Homing Attained
//       {
//         printf("Slave[%d]: Homing Attained.\n", slave_idx);
//       }
//       if ((tx_pdo[slave_idx].status_word >> 13) & 0x01)           // Homing Error
//       {
//         printf("Slave[%d]: Homing Error.\n", slave_idx);
//       }
//       break;
//
//     case 8:   // CYCLIC SYNCHRONOUS POSITION
//       if ((tx_pdo[slave_idx].status_word >> 10) & 0x01)           // Target Reached
//       {
//         printf("Slave[%d]: Target Reached.\n", slave_idx);
//       }
//       if ((tx_pdo[slave_idx].status_word >> 11) & 0x01)           // Internal Limit Active
//       {
//         printf("Slave[%d]: Internal Limit Active\n", slave_idx);
//       }
//       if ((tx_pdo[slave_idx].status_word >> 13) & 0x01)           // Following Error
//       {
//         printf("Slave[%d]: Following Error.\n", slave_idx);
//       }
//       break;
//
//     case 9:   // CYCLIC SYNCHRONOUS VELOCITY
//       if ((tx_pdo[slave_idx].status_word >> 10) & 0x01)           // Target Reached
//       {
//         printf("Slave[%d]: Target Reached.\n", slave_idx);
//       }
//       if ((tx_pdo[slave_idx].status_word >> 11) & 0x01)           // Internal Limit Active
//       {
//         printf("Slave[%d]: Internal Limit Active\n", slave_idx);
//       }
//       if ((tx_pdo[slave_idx].status_word >> 13) & 0x01)           // Following Error
//       {
//         printf("Slave[%d]: Following Error.\n", slave_idx);
//       }
//       break;
//
//     default:
//
//       break;
//   }
// }


inline void print_operation_mode(uint16 slave, int8 operation_mode)
{
  switch (operation_mode)
  {
    case -1:
      printf("%s: Mode of Operation: %s\n", ec_slave[slave].name, "Q Program (ESA specific mode)");
      break;
    case 1:
      printf("%s: Mode of Operation: %s\n", ec_slave[slave].name, "PP (Profile Position)");
      break;
    case 3:
      printf("%s: Mode of Operation: %s\n", ec_slave[slave].name, "PV (Profile Velocity)");
      break;
    case 4:
      printf("%s: Mode of Operation: %s\n", ec_slave[slave].name, "TQ (Torque Profile)");
      break;
    case 6:
      printf("%s: Mode of Operation: %s\n", ec_slave[slave].name, "HM (Homing)");
      break;
    case 8:
      printf("%s: Mode of Operation: %s\n", ec_slave[slave].name, "CSP (Cyclic Synchronous Position)");
      break;
    case 9:
      printf("%s: Mode of Operation: %s\n", ec_slave[slave].name, "CSV (Cyclic Synchronous Velocity)");
      break;
  }
}


inline void print_status_word(uint16 slave, uint16 status_word)
{
  // Ready to Switch On
  if ((status_word >> 0) & 0x01)
  {
    printf("%s: %s", ec_slave[slave].name, "Ready to Swith On");
  }
  // Switched On
  if ((status_word >> 1) & 0x01)
  {
    printf("%s: %s", ec_slave[slave].name, "Switched On");
  }
  // Operation Enabled
  if ((status_word >> 2) & 0x01)
  {
    printf("%s: %s", ec_slave[slave].name, "Operation Enabled");
  }
  // Fault
  if ((status_word >> 3) & 0x01)
  {
    printf("%s: %s", ec_slave[slave].name, "Fault");
  }
  // Voltage Enabled
  if ((status_word >> 4) & 0x01)
  {
    printf("%s: %s", ec_slave[slave].name, "Voltage Enabled");
  }
  // Quick Stop
  if ((status_word >> 5) & 0x01)
  {
    printf("%s: %s", ec_slave[slave].name, "Quick Stop");
  }
  // Switch On Disabled
  if ((status_word >> 6) & 0x01)
  {
    printf("%s: %s", ec_slave[slave].name, "Switch On Disabled");
  }
  // Warning
  if ((status_word >> 7) & 0x01)
  {
    printf("%s: %s", ec_slave[slave].name, "Warning");
  }
  // Remote
  if ((status_word >> 9) & 0x01)
  {
    printf("%s: %s", ec_slave[slave].name, "Remote");
  }
  // Target Reached
  if ((status_word >> 10) & 0x01)
  {
    printf("%s: %s", ec_slave[slave].name, "Target Reached");
  }
  // Internal Limit Active
  if ((status_word >> 11) & 0x01)
  {
    printf("%s: %s", ec_slave[slave].name, "Internal Limit Active");
  }
  // Set Point ACK
  if (!(status_word >> 12) & 0x01)
  {
    printf("%s: %s", ec_slave[slave].name, "Previous set point already processed, waiting for new set point");
  }
  else
  {
    printf("%s: %s", ec_slave[slave].name, "Previous set point still in process, set point overwriting shall be accepted");
  }
  // Following Error
  if (!(status_word >> 13) & 0x01)
  {
    printf("%s: %s", ec_slave[slave].name, "No following error");
  }
  else
  {
    printf("%s: %s", ec_slave[slave].name, "Following Error");
  }
}


inline void print_error_code(uint16 slave, uint16 error_code)
{
  printf("%s: Error Code: 0x%.4x %s\n", ec_slave[slave].name, error_code, error_codes[error_code].c_str());
}


inline void print_alarm_code(uint16 slave, uint32 alarm_code)
{
  printf("%s: Alarm Code: 0x%.8x %s\n", ec_slave[slave].name, alarm_code, alarm_codes[alarm_code].c_str());
}

} } } // namespace
#endif
