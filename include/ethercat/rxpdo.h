#ifndef ESA_EWDL_ETHERCAT_RXPDO_H
#define ESA_EWDL_ETHERCAT_RXPDO_H


namespace esa { namespace ewdl { namespace ethercat { namespace pdo {


struct RxPDO1
{
  uint16 control_word;
  int8 mode_of_operation;
  int32 target_position;
  uint16 touch_probe_function;
  uint32 physical_outputs;

  void operator>>(uint8 *data_ptr)
  {
    *data_ptr++ = (control_word >> 0) & 0xFF;
    *data_ptr++ = (control_word >> 8) & 0xFF;

    *data_ptr++ = (mode_of_operation >> 0) & 0xFF;

    *data_ptr++ = (target_position >> 0) & 0xFF;
    *data_ptr++ = (target_position >> 8) & 0xFF;
    *data_ptr++ = (target_position >> 16) & 0xFF;
    *data_ptr++ = (target_position >> 24) & 0xFF;

    *data_ptr++ = (touch_probe_function >> 0) & 0xFF;
    *data_ptr++ = (touch_probe_function >> 8) & 0xFF;

    *data_ptr++ = (physical_outputs >> 0) & 0xFF;
    *data_ptr++ = (physical_outputs >> 8) & 0xFF;
    *data_ptr++ = (physical_outputs >> 16) & 0xFF;
    *data_ptr++ = (physical_outputs >> 24) & 0xFF;
  }

  // std::stream operator <<(std::stream &os, const RxPDO0 &obj)
  // {
  //   os << "RxPDO0:\n");
  //   printf("\tErrorCode: %x\n", rx_pdo.error_code);
  //   printf("\tStatusWord: %d\n", rx_pdo.status_word);
  //   printf("\tModes of Operation Display: %d\n", rx_pdo.modes_of_operation_display);
  //   printf("\tPosition Actual Value: %d\n", rx_pdo.position_actual_value);
  //   printf("\tFollow Error Actual Value: %d\n", rx_pdo.follow_error_actual_value);
  //   printf("\tTouch Probe Status: %d\n", rx_pdo.touch_probe_status);
  //   printf("\tTouch probe pos1 neg value: %d\n", rx_pdo.touch_probe_pos1_neg_value);
  //   printf("\tTouch probe pos1 pos value: %d\n", rx_pdo.touch_probe_pos1_pos_value);
  //   printf("\tTouch probe pos2 pos value: %d\n", rx_pdo.touch_probe_pos2_pos_value);
  //   printf("\tTouch probe pos2 neg value: %d\n", rx_pdo.touch_probe_pos2_neg_value);
  //   printf("\tDigital Inputs: %.32x\n", rx_pdo.digital_inputs);
  // }

};


struct RxPDO2
{
  uint16 control_word;
  int8 mode_of_operation;
  int32 target_velocity;
  uint16 touch_probe_function;
  uint32 physical_outputs;

  void operator>>(uint8 *data_ptr)
  {
    *data_ptr++ = (control_word >> 0) & 0xFF;
    *data_ptr++ = (control_word >> 8) & 0xFF;

    *data_ptr++ = (mode_of_operation >> 0) & 0xFF;

    *data_ptr++ = (target_velocity >> 0) & 0xFF;
    *data_ptr++ = (target_velocity >> 8) & 0xFF;
    *data_ptr++ = (target_velocity >> 16) & 0xFF;
    *data_ptr++ = (target_velocity >> 24) & 0xFF;

    *data_ptr++ = (touch_probe_function >> 0) & 0xFF;
    *data_ptr++ = (touch_probe_function >> 8) & 0xFF;

    *data_ptr++ = (physical_outputs >> 0) & 0xFF;
    *data_ptr++ = (physical_outputs >> 8) & 0xFF;
    *data_ptr++ = (physical_outputs >> 16) & 0xFF;
    *data_ptr++ = (physical_outputs >> 24) & 0xFF;
  }

  // std::stream operator <<(std::stream &os, const T &obj)
  // {
  //   os << "RxPDO0:\n");
  //   printf("\tErrorCode: %x\n", rx_pdo.error_code);
  //   printf("\tStatusWord: %d\n", rx_pdo.status_word);
  //   printf("\tModes of Operation Display: %d\n", rx_pdo.modes_of_operation_display);
  //   printf("\tPosition Actual Value: %d\n", rx_pdo.position_actual_value);
  //   printf("\tFollow Error Actual Value: %d\n", rx_pdo.follow_error_actual_value);
  //   printf("\tTouch Probe Status: %d\n", rx_pdo.touch_probe_status);
  //   printf("\tTouch probe pos1 neg value: %d\n", rx_pdo.touch_probe_pos1_neg_value);
  //   printf("\tTouch probe pos1 pos value: %d\n", rx_pdo.touch_probe_pos1_pos_value);
  //   printf("\tTouch probe pos2 pos value: %d\n", rx_pdo.touch_probe_pos2_pos_value);
  //   printf("\tTouch probe pos2 neg value: %d\n", rx_pdo.touch_probe_pos2_neg_value);
  //   printf("\tDigital Inputs: %.32x\n", rx_pdo.digital_inputs);
  // }

};


struct RxPDO3
{
  uint16 control_word;
  int8 mode_of_operation;
  int32 target_position;
  uint32 profile_velocity;
  uint32 physical_outputs;

  void operator>>(uint8 *data_ptr)
  {
    *data_ptr++ = (control_word >> 0) & 0xFF;
    *data_ptr++ = (control_word >> 8) & 0xFF;

    *data_ptr++ = (mode_of_operation >> 0) & 0xFF;

    *data_ptr++ = (target_position >> 0) & 0xFF;
    *data_ptr++ = (target_position >> 8) & 0xFF;
    *data_ptr++ = (target_position >> 16) & 0xFF;
    *data_ptr++ = (target_position >> 24) & 0xFF;

    *data_ptr++ = (profile_velocity >> 0) & 0xFF;
    *data_ptr++ = (profile_velocity >> 8) & 0xFF;
    *data_ptr++ = (profile_velocity >> 16) & 0xFF;
    *data_ptr++ = (profile_velocity >> 24) & 0xFF;

    *data_ptr++ = (physical_outputs >> 0) & 0xFF;
    *data_ptr++ = (physical_outputs >> 8) & 0xFF;
    *data_ptr++ = (physical_outputs >> 16) & 0xFF;
    *data_ptr++ = (physical_outputs >> 24) & 0xFF;
  }

  // std::stream operator <<(std::stream &os, const T &obj)
  // {
  //   os << "RxPDO0:\n");
  //   printf("\tErrorCode: %x\n", rx_pdo.error_code);
  //   printf("\tStatusWord: %d\n", rx_pdo.status_word);
  //   printf("\tModes of Operation Display: %d\n", rx_pdo.modes_of_operation_display);
  //   printf("\tPosition Actual Value: %d\n", rx_pdo.position_actual_value);
  //   printf("\tFollow Error Actual Value: %d\n", rx_pdo.follow_error_actual_value);
  //   printf("\tTouch Probe Status: %d\n", rx_pdo.touch_probe_status);
  //   printf("\tTouch probe pos1 neg value: %d\n", rx_pdo.touch_probe_pos1_neg_value);
  //   printf("\tTouch probe pos1 pos value: %d\n", rx_pdo.touch_probe_pos1_pos_value);
  //   printf("\tTouch probe pos2 pos value: %d\n", rx_pdo.touch_probe_pos2_pos_value);
  //   printf("\tTouch probe pos2 neg value: %d\n", rx_pdo.touch_probe_pos2_neg_value);
  //   printf("\tDigital Inputs: %.32x\n", rx_pdo.digital_inputs);
  // }

};


struct RxPDO4
{
  uint16 control_word;
  int8 mode_of_operation;
  int32 target_velocity;
  uint32 physical_outputs;

  void operator>>(uint8 *data_ptr)
  {
    *data_ptr++ = (control_word >> 0) & 0xFF;
    *data_ptr++ = (control_word >> 8) & 0xFF;

    *data_ptr++ = (mode_of_operation >> 0) & 0xFF;

    *data_ptr++ = (target_velocity >> 0) & 0xFF;
    *data_ptr++ = (target_velocity >> 8) & 0xFF;
    *data_ptr++ = (target_velocity >> 16) & 0xFF;
    *data_ptr++ = (target_velocity >> 24) & 0xFF;

    *data_ptr++ = (physical_outputs >> 0) & 0xFF;
    *data_ptr++ = (physical_outputs >> 8) & 0xFF;
    *data_ptr++ = (physical_outputs >> 16) & 0xFF;
    *data_ptr++ = (physical_outputs >> 24) & 0xFF;
  }

  // std::stream operator <<(std::stream &os, const T &obj)
  // {
  //   os << "RxPDO0:\n");
  //   printf("\tErrorCode: %x\n", rx_pdo.error_code);
  //   printf("\tStatusWord: %d\n", rx_pdo.status_word);
  //   printf("\tModes of Operation Display: %d\n", rx_pdo.modes_of_operation_display);
  //   printf("\tPosition Actual Value: %d\n", rx_pdo.position_actual_value);
  //   printf("\tFollow Error Actual Value: %d\n", rx_pdo.follow_error_actual_value);
  //   printf("\tTouch Probe Status: %d\n", rx_pdo.touch_probe_status);
  //   printf("\tTouch probe pos1 neg value: %d\n", rx_pdo.touch_probe_pos1_neg_value);
  //   printf("\tTouch probe pos1 pos value: %d\n", rx_pdo.touch_probe_pos1_pos_value);
  //   printf("\tTouch probe pos2 pos value: %d\n", rx_pdo.touch_probe_pos2_pos_value);
  //   printf("\tTouch probe pos2 neg value: %d\n", rx_pdo.touch_probe_pos2_neg_value);
  //   printf("\tDigital Inputs: %.32x\n", rx_pdo.digital_inputs);
  // }

};


// union RxPDO
// {
//   RxPDO1 _0;
//   RxPDO2 _1;
//   RxPDO3 _2;
//   RxPDO4 _3;
// };


} } } } // namespace
#endif
