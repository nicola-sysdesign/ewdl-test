#ifndef ESA_EWDL_ETHERCAT_TXPDO_H
#define ESA_EWDL_ETHERCAT_TXPDO_H


namespace esa { namespace ewdl { namespace ethercat { namespace pdo {


struct TxPDO1
{
  uint16 error_code;
  uint16 status_word;
  int8 mode_of_operation_display;
  int32 position_actual_value;
  int32 follow_error_actual_value;
  uint16 touch_probe_status;
  int32 touch_probe_pos1_pos_value;
  int32 touch_probe_pos1_neg_value;
  int32 touch_probe_pos2_pos_value;
  int32 touch_probe_pos2_neg_value;
  uint32 digital_inputs;

  void operator<<(uint8 *data_ptr)
  {
    error_code = 0x0000;
    status_word = 0x0000;
    mode_of_operation_display = 0x00;
    position_actual_value = 0x00000000;
    follow_error_actual_value = 0x00000000;
    touch_probe_status = 0x0000;
    touch_probe_pos1_pos_value = 0x00000000;
    touch_probe_pos1_neg_value = 0x00000000;
    touch_probe_pos2_pos_value = 0x00000000;
    touch_probe_pos2_neg_value = 0x00000000;
    digital_inputs = 0x00000000;

    error_code |= (0x00FF & *data_ptr++) << 0;
    error_code |= (0x00FF & *data_ptr++) << 8;

    status_word |= (0x00FF & *data_ptr++) << 0;
    status_word |= (0x00FF & *data_ptr++) << 8;

    mode_of_operation_display |= (0xFF & *data_ptr++) << 0;

    position_actual_value |= (0x000000FF & *data_ptr++) << 0;
    position_actual_value |= (0x000000FF & *data_ptr++) << 8;
    position_actual_value |= (0x000000FF & *data_ptr++) << 16;
    position_actual_value |= (0x000000FF & *data_ptr++) << 24;

    follow_error_actual_value |= (0x000000FF & *data_ptr++) << 0;
    follow_error_actual_value |= (0x000000FF & *data_ptr++) << 8;
    follow_error_actual_value |= (0x000000FF & *data_ptr++) << 16;
    follow_error_actual_value |= (0x000000FF & *data_ptr++) << 24;

    touch_probe_status |= (0x00FF & *data_ptr++) << 0;
    touch_probe_status |= (0x00FF & *data_ptr++) << 8;

    touch_probe_pos1_pos_value |= (0x000000FF & *data_ptr++) << 0;
    touch_probe_pos1_pos_value |= (0x000000FF & *data_ptr++) << 8;
    touch_probe_pos1_pos_value |= (0x000000FF & *data_ptr++) << 16;
    touch_probe_pos1_pos_value |= (0x000000FF & *data_ptr++) << 24;

    touch_probe_pos1_neg_value |= (0x000000FF & *data_ptr++) << 0;
    touch_probe_pos1_neg_value |= (0x000000FF & *data_ptr++) << 8;
    touch_probe_pos1_neg_value |= (0x000000FF & *data_ptr++) << 16;
    touch_probe_pos1_neg_value |= (0x000000FF & *data_ptr++) << 24;

    touch_probe_pos2_pos_value |= (0x000000FF & *data_ptr++) << 0;
    touch_probe_pos2_pos_value |= (0x000000FF & *data_ptr++) << 8;
    touch_probe_pos2_pos_value |= (0x000000FF & *data_ptr++) << 16;
    touch_probe_pos2_pos_value |= (0x000000FF & *data_ptr++) << 24;

    touch_probe_pos2_neg_value |= (0x000000FF & *data_ptr++) << 0;
    touch_probe_pos2_neg_value |= (0x000000FF & *data_ptr++) << 8;
    touch_probe_pos2_neg_value |= (0x000000FF & *data_ptr++) << 16;
    touch_probe_pos2_neg_value |= (0x000000FF & *data_ptr++) << 24;

    digital_inputs |= (0x000000FF & *data_ptr++) << 0;
    digital_inputs |= (0x000000FF & *data_ptr++) << 8;
    digital_inputs |= (0x000000FF & *data_ptr++) << 16;
    digital_inputs |= (0x000000FF & *data_ptr++) << 24;
  }

  // std::stream operator <<(std::stream &os, const T &obj) {
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


struct TxPDO2
{
  uint16 error_code;
  uint16 status_word;
  int8 mode_of_operation_display;
  int32 position_actual_value;
  int32 velocity_actual_value;
  int32 follow_error_actual_value;
  uint16 touch_probe_status;
  int32 touch_probe_pos1_pos_value;
  int32 touch_probe_pos1_neg_value;
  int32 touch_probe_pos2_pos_value;
  int32 touch_probe_pos2_neg_value;
  uint32 digital_inputs;

  void operator<<(uint8 *data_ptr)
  {
    error_code = 0x0000;
    status_word = 0x0000;
    mode_of_operation_display = 0x00;
    position_actual_value = 0x00000000;
    velocity_actual_value = 0x00000000;
    follow_error_actual_value = 0x00000000;
    touch_probe_status = 0x0000;
    touch_probe_pos1_pos_value = 0x00000000;
    touch_probe_pos1_neg_value = 0x00000000;
    touch_probe_pos2_pos_value = 0x00000000;
    touch_probe_pos2_neg_value = 0x00000000;
    digital_inputs = 0x00000000;

    error_code |= (0x00FF & *data_ptr++) << 0;
    error_code |= (0x00FF & *data_ptr++) << 8;

    status_word |= (0x00FF & *data_ptr++) << 0;
    status_word |= (0x00FF & *data_ptr++) << 8;

    mode_of_operation_display |= (0xFF & *data_ptr++) << 0;

    position_actual_value |= (0x000000FF & *data_ptr++) << 0;
    position_actual_value |= (0x000000FF & *data_ptr++) << 8;
    position_actual_value |= (0x000000FF & *data_ptr++) << 16;
    position_actual_value |= (0x000000FF & *data_ptr++) << 24;

    velocity_actual_value |= (0x000000FF & *data_ptr++) << 0;
    velocity_actual_value |= (0x000000FF & *data_ptr++) << 8;
    velocity_actual_value |= (0x000000FF & *data_ptr++) << 16;
    velocity_actual_value |= (0x000000FF & *data_ptr++) << 24;

    follow_error_actual_value |= (0x000000FF & *data_ptr++) << 0;
    follow_error_actual_value |= (0x000000FF & *data_ptr++) << 8;
    follow_error_actual_value |= (0x000000FF & *data_ptr++) << 16;
    follow_error_actual_value |= (0x000000FF & *data_ptr++) << 24;

    touch_probe_status |= (0x00FF & *data_ptr++) << 0;
    touch_probe_status |= (0x00FF & *data_ptr++) << 8;

    touch_probe_pos1_pos_value |= (0x000000FF & *data_ptr++) << 0;
    touch_probe_pos1_pos_value |= (0x000000FF & *data_ptr++) << 8;
    touch_probe_pos1_pos_value |= (0x000000FF & *data_ptr++) << 16;
    touch_probe_pos1_pos_value |= (0x000000FF & *data_ptr++) << 24;

    touch_probe_pos1_neg_value |= (0x000000FF & *data_ptr++) << 0;
    touch_probe_pos1_neg_value |= (0x000000FF & *data_ptr++) << 8;
    touch_probe_pos1_neg_value |= (0x000000FF & *data_ptr++) << 16;
    touch_probe_pos1_neg_value |= (0x000000FF & *data_ptr++) << 24;

    touch_probe_pos2_pos_value |= (0x000000FF & *data_ptr++) << 0;
    touch_probe_pos2_pos_value |= (0x000000FF & *data_ptr++) << 8;
    touch_probe_pos2_pos_value |= (0x000000FF & *data_ptr++) << 16;
    touch_probe_pos2_pos_value |= (0x000000FF & *data_ptr++) << 24;

    touch_probe_pos2_neg_value |= (0x000000FF & *data_ptr++) << 0;
    touch_probe_pos2_neg_value |= (0x000000FF & *data_ptr++) << 8;
    touch_probe_pos2_neg_value |= (0x000000FF & *data_ptr++) << 16;
    touch_probe_pos2_neg_value |= (0x000000FF & *data_ptr++) << 24;

    digital_inputs |= (0x000000FF & *data_ptr++) << 0;
    digital_inputs |= (0x000000FF & *data_ptr++) << 8;
    digital_inputs |= (0x000000FF & *data_ptr++) << 16;
    digital_inputs |= (0x000000FF & *data_ptr++) << 24;
  }

  // std::stream operator <<(std::stream &os, const T &obj) {
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


struct TxPDO3
{
  uint16 error_code;
  uint16 status_word;
  int8 mode_of_operation_display;
  int32 position_actual_value;
  int32 velocity_actual_value;
  int32 follow_error_actual_value;
  uint32 digital_inputs;

  void operator<<(uint8 *data_ptr)
  {
    error_code = 0x0000;
    status_word = 0x0000;
    mode_of_operation_display = 0x00;
    position_actual_value = 0x00000000;
    velocity_actual_value = 0x00000000;
    follow_error_actual_value = 0x00000000;
    digital_inputs = 0x00000000;

    error_code |= (0x00FF & *data_ptr++) << 0;
    error_code |= (0x00FF & *data_ptr++) << 8;

    status_word |= (0x00FF & *data_ptr++) << 0;
    status_word |= (0x00FF & *data_ptr++) << 8;

    mode_of_operation_display |= (0xFF & *data_ptr++) << 0;

    position_actual_value |= (0x000000FF & *data_ptr++) << 0;
    position_actual_value |= (0x000000FF & *data_ptr++) << 8;
    position_actual_value |= (0x000000FF & *data_ptr++) << 16;
    position_actual_value |= (0x000000FF & *data_ptr++) << 24;

    velocity_actual_value |= (0x000000FF & *data_ptr++) << 0;
    velocity_actual_value |= (0x000000FF & *data_ptr++) << 8;
    velocity_actual_value |= (0x000000FF & *data_ptr++) << 16;
    velocity_actual_value |= (0x000000FF & *data_ptr++) << 24;

    follow_error_actual_value |= (0x000000FF & *data_ptr++) << 0;
    follow_error_actual_value |= (0x000000FF & *data_ptr++) << 8;
    follow_error_actual_value |= (0x000000FF & *data_ptr++) << 16;
    follow_error_actual_value |= (0x000000FF & *data_ptr++) << 24;

    digital_inputs |= (0x000000FF & *data_ptr++) << 0;
    digital_inputs |= (0x000000FF & *data_ptr++) << 8;
    digital_inputs |= (0x000000FF & *data_ptr++) << 16;
    digital_inputs |= (0x000000FF & *data_ptr++) << 24;
  }

  // std::stream operator <<(std::stream &os, const T &obj) {
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


struct TxPDO4
{
  uint16 error_code;
  uint16 status_word;
  int8 mode_of_operation_display;
  int32 position_actual_value;
  int32 velocity_actual_value;
  int32 follow_error_actual_value;
  uint32 digital_inputs;

  void operator<<(uint8 *data_ptr)
  {
    error_code = 0x0000;
    status_word = 0x0000;
    mode_of_operation_display = 0x00;
    position_actual_value = 0x00000000;
    velocity_actual_value = 0x00000000;
    follow_error_actual_value = 0x00000000;
    digital_inputs = 0x00000000;

    error_code |= (0x00FF & *data_ptr++) << 0;
    error_code |= (0x00FF & *data_ptr++) << 8;

    status_word |= (0x00FF & *data_ptr++) << 0;
    status_word |= (0x00FF & *data_ptr++) << 8;

    mode_of_operation_display |= (0xFF & *data_ptr++) << 0;

    position_actual_value |= (0x000000FF & *data_ptr++) << 0;
    position_actual_value |= (0x000000FF & *data_ptr++) << 8;
    position_actual_value |= (0x000000FF & *data_ptr++) << 16;
    position_actual_value |= (0x000000FF & *data_ptr++) << 24;

    velocity_actual_value |= (0x000000FF & *data_ptr++) << 0;
    velocity_actual_value |= (0x000000FF & *data_ptr++) << 8;
    velocity_actual_value |= (0x000000FF & *data_ptr++) << 16;
    velocity_actual_value |= (0x000000FF & *data_ptr++) << 24;

    follow_error_actual_value |= (0x000000FF & *data_ptr++) << 0;
    follow_error_actual_value |= (0x000000FF & *data_ptr++) << 8;
    follow_error_actual_value |= (0x000000FF & *data_ptr++) << 16;
    follow_error_actual_value |= (0x000000FF & *data_ptr++) << 24;

    digital_inputs |= (0x000000FF & *data_ptr++) << 0;
    digital_inputs |= (0x000000FF & *data_ptr++) << 8;
    digital_inputs |= (0x000000FF & *data_ptr++) << 16;
    digital_inputs |= (0x000000FF & *data_ptr++) << 24;
  }

  // std::stream operator <<(std::stream &os, const T &obj) {
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


// union TxPDO
// {
//   TxPDO0 _0;
//   TxPDO1 _1;
//   TxPDO2 _2;
//   TxPDO3 _3;
// };


} } } } // namespace
#endif
