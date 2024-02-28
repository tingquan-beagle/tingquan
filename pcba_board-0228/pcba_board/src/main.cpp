#include <Arduino.h>  // Essential

#include "CANbus.h"
#include "cylinder.h"
#include "fourTwenty.h"
#include "gripper.h"
#include "hardwarePins.h"
#include "joystick.cpp"
#include "microSD.h"
#include "peripherals.h"  // Essential
#include "relay.h"

/*----------------DO NOT REMOVE----------------------------------------*/
// used by fourTwenty and microSD
SPIClass hspi = SPIClass(HSPI);
SPIClass fspi = SPIClass(FSPI);

void initSPI();
/*----------------DO NOT REMOVE----------------------------------------*/

/*---------------------PREPROCESSOR CONFIGURATION---------------------*/

// NOTE: SEE platformio.ini for PLATFORM CONFIGURATION

#define SERIAL_DEBUG_SEND_FLG false
#define SERIAL_DEBUG_AUTO_FLG false
#define SERIAL_DEBUG_MANU_FLG false
#define SERIAL_GENERAL true

enum Platform { GRAPE = 0, CELERY = 1, DANFOSS = 2 };

void setupSerial();

/*---------------------PREPROCESSOR CONFIGURATION---------------------*/

/*--------------------------GRAPE ASSIGNMENTS--------------------------*/
// PINS
#define LEFT_VER_CYL 1
#define RIGHT_VER_CYL 3
#define LEFT_HOR_CYL 2
#define RIGHT_HOR_CYL 4

// CAN ADDRESSES
#define CAN_MANUAL_ID 0x002

#define CAN_JOYSTICK_DLC 8

// OBJECT INSTANCES
CylinderHydraulic vertical_left_cyl(LEFT_VER_CYL);
CylinderHydraulic vertical_right_cyl(RIGHT_VER_CYL);

Joystick joystick(&vertical_left_cyl, &vertical_right_cyl);

// VARIABLES
uint8_t can_joystick_msg[CAN_JOYSTICK_DLC];

/*--------------------------GRAPE ASSIGNMENTS--------------------------*/

/*--------------------------CELREY ASSIGNMENTS--------------------------*/
// PINS
#define GRIPPER_POS 1   // gripper pos
#define GRIPPER_NEG 3   // gripper neg
#define PIN_ROTATION 2  // rotational valve

// CAN ADDRESSES
#define CAN_CELERY_ID 0x020

#define CAN_CELERY_STAT_ID 0x05
#define CAN_CELERY_STAT_DLC 8
#define CAN_CELERY_STAT_MS 100

// OBJECT INSTANCES
CylinderPneumatic vertical_left_cyl_cel(LEFT_VER_CYL);
CylinderPneumatic vertical_right_cyl_cel(RIGHT_VER_CYL);
CylinderPneumatic horizontal_right_cyl(RIGHT_HOR_CYL);

Gripper right_gripper(GRIPPER_NEG, GRIPPER_POS, PIN_ROTATION,
                      &horizontal_right_cyl, &vertical_right_cyl_cel);

// VARIABLES
int flag_celery_trigger = 0;
uint8_t can_celery_stat_msg[CAN_CELERY_STAT_DLC];

/*--------------------------CELERY ASSIGNMENTS--------------------------*/

/*------------------------------ENUMERATORS----------------------------*/
enum CAN_Auto_frame {
  CONF_IDX = 0,
  CMD_IDX_UPPER = 1,
  CMD_IDX_LOWER = 2,
  FRM_CNT_IDX = 3,
  CELERY_TRIG_IDX = 4,
};

enum CAN_Manual_frame {
  RIGHT_ENCODER_IDX = 0,
  LEFT_ENCODER_IDX = 1,
  BINARY_COMPONENTS_IDX = 2
};

enum CAN_Manual_component {
  RIGHT_BTT_IDX = 0,
  LEFT_BTT_IDX = 1,
  RIGHT_SW_IDX = 2,
  LEFT_SW_IDX = 3
};
/*------------------------------ENUMERATORS----------------------------*/

/*----------------------------CUSTOM DATA TYPES--------------------------*/
struct as_bytes_t {
  uint8_t lower;
  uint8_t upper;
};

union position_t {
  as_bytes_t as_byte;
  uint16_t as_word;
};
/*----------------------------CUSTOM DATA TYPES--------------------------*/

/*----------------------------COMMON--------------------------*/

// CAN ADDRESSES
#define CAN_LEFT_CAM_ID 0x014
#define CAN_RIGHT_CAM_ID 0x015

#define CAN_LOGGING_ID 0x003
#define CAN_LOGGING_DLC 6
#define CAN_LOGGING_MS 200

#define CAN_POSITION_ID 0X04
#define CAN_POSITION_DLC 8
#define CAN_POSITION_MS 50

#define CAN_MODE_ID 0x002
#define CAN_MODE_DLC 3
#define CAN_MODE_MS 50

// CAN STRUCTURE

struct CanMessage {
  void (*publishFunc)();        // Function to publish the CAN message
  unsigned long interval;       // Time interval in milliseconds
  unsigned long lastTimestamp;  // Timestamp of the last publication
};

// VARIABLES
bool left_btt_state, right_btt_state;
bool left_sw_state, right_sw_state;

static uint8_t left_frame_count, right_frame_count;

uint8_t can_log_msg[CAN_LOGGING_DLC];
uint8_t can_position_msg[CAN_POSITION_DLC];
uint8_t can_mode_msg[CAN_MODE_DLC];

inline bool get_bit(uint8_t n, uint8_t k) { return (n >> k) & 1; }
/*----------------------------COMMON--------------------------*/

void publishIfTimeElapsed(CanMessage &canMessage) {
  if ((millis() - canMessage.lastTimestamp) > canMessage.interval) {
    canMessage.publishFunc();
    canMessage.lastTimestamp = millis();
  }
}

void can_log_publish() {
  int index = 0;
  can_log_msg[index] = vertical_left_cyl.getLastConf();
  can_log_msg[++index] = vertical_left_cyl.getLastCmd();
  can_log_msg[++index] = vertical_left_cyl.getLastPosition();
  can_log_msg[++index] = vertical_right_cyl.getLastConf();
  can_log_msg[++index] = vertical_right_cyl.getLastCmd();
  can_log_msg[++index] = vertical_right_cyl.getLastPosition();

  CANsend(CAN_LOGGING_ID, can_log_msg);

#if SERIAL_DEBUG_SEND_FLG
  serial_publish_display(&can_log_msg);
#endif
}

void can_position_publish() {
  union position_t left_position;
  union position_t right_position;
  union position_t left_cmd;
  union position_t right_cmd;
  left_cmd.as_word = uint16_t(vertical_left_cyl.getLastCmd());
  right_cmd.as_word = uint16_t(vertical_right_cyl.getLastCmd());
  left_position.as_word = uint16_t(0);
  right_position.as_word = uint16_t(0);

  int index = 0;
  can_position_msg[index] = left_position.as_byte.upper;
  can_position_msg[++index] = left_position.as_byte.lower;
  can_position_msg[++index] = left_cmd.as_byte.upper;
  can_position_msg[++index] = left_cmd.as_byte.lower;
  can_position_msg[++index] = right_position.as_byte.upper;
  can_position_msg[++index] = right_position.as_byte.lower;
  can_position_msg[++index] = right_cmd.as_byte.upper;
  can_position_msg[++index] = right_cmd.as_byte.lower;

  CANsend(CAN_POSITION_ID, can_position_msg);

#if SERIAL_DEBUG_SEND_FLG
  serial_publish_display(&can_position_msg);
#endif
}

void can_celery_stat_publish() {
  int gripper_state = right_gripper.getGripperState();
  unsigned long gripper_timestamp = right_gripper.getSeqTimestamp();
  int deadman_counter = joystick.getDeadmanClickCount();

  gripper_state = right_gripper.getGripperState();
  gripper_timestamp = right_gripper.getSeqTimestamp();

  int index = 0;
  can_celery_stat_msg[index] = vertical_right_cyl.getLastPosition();
  can_celery_stat_msg[++index] = gripper_state;
  can_celery_stat_msg[++index] = (gripper_timestamp >> 24) & 0xFF;
  can_celery_stat_msg[++index] = (gripper_timestamp >> 16) & 0xFF;
  can_celery_stat_msg[++index] = (gripper_timestamp >> 8) & 0xFF;
  can_celery_stat_msg[++index] = gripper_timestamp & 0xFF;
  can_celery_stat_msg[++index] = deadman_counter;

  CANsend(CAN_CELERY_STAT_ID, can_celery_stat_msg);

#if SERIAL_DEBUG_SEND_FLG
  serial_publish_display(&can_celery_stat_msg);
#endif
}

void can_mode_publish() {
  bool left_auto_state = joystick.isLeftAutomaticMode();
  bool right_auto_state = joystick.isRightAutomaticMode();

  can_mode_msg[RIGHT_ENCODER_IDX] = 0x00;
  can_mode_msg[LEFT_ENCODER_IDX] = 0x00;
  can_mode_msg[BINARY_COMPONENTS_IDX] = 0x00 |
                                        (right_auto_state << RIGHT_SW_IDX) |
                                        (left_auto_state << LEFT_SW_IDX);

  CANsend(CAN_MODE_ID, can_mode_msg);

#if SERIAL_DEBUG_SEND_FLG
  serial_publish_display();
#endif
}

// can comm for servo motor control, PDO mode
const uint16_t RPDO1 = 0x200;
const uint16_t RPDO2 = 0x300;
const uint16_t RPDO3 = 0x400;
const uint16_t RPDO4 = 0x500;
const uint16_t ServoNodeID = 0x0A;
const uint16_t TPDO1 = 0x180;
const uint16_t TPDO2 = 0x280;
const uint16_t TPDO3 = 0x380;
const uint16_t TPDO4 = 0x480;

void can_nmt_start_publish() {
  static uint8_t can_nmt_start_msg[2] = {0x01, 0x00};
  can_nmt_start_msg[1] = ServoNodeID;
  int ret = CANsend(0x0, can_nmt_start_msg,
                    sizeof(can_nmt_start_msg) / sizeof(uint8_t));
#if SERIAL_DEBUG_SEND_FLG or SERIAL_DEBUG_AUTO_FLG or SERIAL_DEBUG_MANU_FLG or \
    SERIAL_GENERAL
  Serial.print("send nmt start return with ");
  Serial.println(ret, HEX);
#endif
}

void can_set_pp_mode() {
  static uint8_t can_pp_mode_msg[1] = {0x01};
  int ret = CANsend(RPDO2 + ServoNodeID, can_pp_mode_msg,
                    sizeof(can_pp_mode_msg) / sizeof(uint8_t));
#if SERIAL_DEBUG_SEND_FLG or SERIAL_DEBUG_AUTO_FLG or SERIAL_DEBUG_MANU_FLG or \
    SERIAL_GENERAL
  Serial.print("send pp mode message return with ");
  Serial.println(ret, HEX);
#endif
}

void can_change_servo_mode() {
  // ready to switch on
  static uint8_t can_ready_to_switch_on_msg[2] = {0x06, 0x00};
  int ret = ESP_OK;
  ret = CANsend(RPDO1 + ServoNodeID, can_ready_to_switch_on_msg,
                sizeof(can_ready_to_switch_on_msg) / sizeof(uint8_t));
#if SERIAL_DEBUG_SEND_FLG or SERIAL_DEBUG_AUTO_FLG or SERIAL_DEBUG_MANU_FLG or \
    SERIAL_GENERAL
  Serial.print("can send ready to switch on msg return with ");
  Serial.println(ret, HEX);
#endif

  static uint8_t can_switch_on_msg[2] = {0x07, 0x00};
  ret = CANsend(RPDO1 + ServoNodeID, can_switch_on_msg,
                sizeof(can_switch_on_msg) / sizeof(uint8_t));
#if SERIAL_DEBUG_SEND_FLG or SERIAL_DEBUG_AUTO_FLG or SERIAL_DEBUG_MANU_FLG or \
    SERIAL_GENERAL
  Serial.print("can send ready to switch on msg return with ");
  Serial.println(ret, HEX);
#endif

  static uint8_t can_operation_enable_msg[2] = {0x07, 0x00};
  ret = CANsend(RPDO1 + ServoNodeID, can_operation_enable_msg,
                sizeof(can_operation_enable_msg) / sizeof(uint8_t));
#if SERIAL_DEBUG_SEND_FLG or SERIAL_DEBUG_AUTO_FLG or SERIAL_DEBUG_MANU_FLG or \
    SERIAL_GENERAL
  Serial.print("can send ready to switch on msg return with ");
  Serial.println(ret, HEX);
#endif
}

void initServo() {
  can_nmt_start_publish();
  can_set_pp_mode();
}

void setServoTarget(uint32_t position, uint32_t speed) {
  static uint8_t can_target_msg[8];
  int index = 0;
  can_target_msg[index] = position & 0xFF;
  can_target_msg[++index] = (position >> 8) & 0xFF;
  can_target_msg[++index] = (position >> 16) & 0xFF;
  can_target_msg[++index] = (position >> 24) & 0xFF;

  can_target_msg[++index] = speed & 0xFF;
  can_target_msg[++index] = (speed >> 8) & 0xFF;
  can_target_msg[++index] = (speed >> 16) & 0xFF;
  can_target_msg[++index] = (speed >> 24) & 0xFF;
  int ret = CANsend(RPDO4 + ServoNodeID, can_target_msg,
                    sizeof(can_target_msg) / sizeof(uint8_t));
#if SERIAL_DEBUG_SEND_FLG or SERIAL_DEBUG_AUTO_FLG or SERIAL_DEBUG_MANU_FLG or \
    SERIAL_GENERAL
  Serial.print("can send target position/speed msg return with ");
  Serial.println(ret, HEX);
#endif
}

enum PPCOMMAND {
  PPSTART = 0x3F,
  PPPAUSE = 0x03,
  PPRESUME = 0x0F,
  PPSTOP = 0x05
};

void runPPcommand(PPCOMMAND cmd) {
  static uint8_t can_pp_command_msg[2] = {0x00, 0x00};
  can_pp_command_msg[0] = cmd;
  int ret = ESP_OK;
  ret = CANsend(RPDO1 + ServoNodeID, can_pp_command_msg,
                sizeof(can_pp_command_msg) / sizeof(uint8_t));
#if SERIAL_DEBUG_SEND_FLG or SERIAL_DEBUG_AUTO_FLG or SERIAL_DEBUG_MANU_FLG or \
    SERIAL_GENERAL
  Serial.print("can execute pp command return with ");
  Serial.println(ret, HEX);
#endif
}
void startPP() { runPPcommand(PPSTART); }
void pausePP() { runPPcommand(PPPAUSE); }
void resumePP() { runPPcommand(PPRESUME); }
void stopPP() { runPPcommand(PPSTOP); }

uint16_t StatusWord = 0;
uint16_t ErrorCode = 0;
uint32_t MotorSpeed = 0;
uint16_t MotorCurrent = 0;
uint32_t MotorRounds = 0;
uint16_t MotorAngleRounds = 0;
uint16_t MotorAngleAngle = 0;
void CAN_TPDO_CALLBACK() {
  twai_message_t can_receive_msg;
  int16_t cmd_val;

  int ret = twai_receive(&can_receive_msg, 0);
  if (ret == ESP_OK) {
    switch (can_receive_msg.identifier) {
      case TPDO1 + ServoNodeID:
        StatusWord = ((uint16_t)(can_receive_msg.data[1]) << 8) +
                     can_receive_msg.data[0];
        break;
      case TPDO2 + ServoNodeID:
        ErrorCode = ((uint16_t)(can_receive_msg.data[1]) << 8) +
                    can_receive_msg.data[0];
        break;
      case TPDO3 + ServoNodeID:
        MotorSpeed = ((uint32_t)(can_receive_msg.data[3]) << 24) +
                     ((uint32_t)(can_receive_msg.data[2]) << 16) +
                     ((uint32_t)(can_receive_msg.data[1]) << 8) +
                     can_receive_msg.data[0];
        MotorCurrent = ((uint32_t)(can_receive_msg.data[5]) << 8) +
                       can_receive_msg.data[4];
        break;
      case TPDO4 + ServoNodeID:
        MotorRounds = ((uint32_t)(can_receive_msg.data[3]) << 24) +
                      ((uint32_t)(can_receive_msg.data[2]) << 16) +
                      ((uint32_t)(can_receive_msg.data[1]) << 8) +
                      can_receive_msg.data[0];
        MotorAngleAngle = ((uint32_t)(can_receive_msg.data[5]) << 8) +
                          can_receive_msg.data[4];
        MotorAngleRounds = ((uint32_t)(can_receive_msg.data[7]) << 8) +
                           can_receive_msg.data[6];
        break;
      default:
#if SERIAL_DEBUG_SEND_FLG or SERIAL_DEBUG_AUTO_FLG or SERIAL_DEBUG_MANU_FLG or \
    SERIAL_GENERAL
        Serial.print("tpdo read unkown ");
        Serial.println(can_receive_msg.identifier, HEX);
#endif
        break;
    }
  } else {
#if SERIAL_DEBUG_SEND_FLG or SERIAL_DEBUG_AUTO_FLG or SERIAL_DEBUG_MANU_FLG or \
    SERIAL_GENERAL
    // Serial.print("twai_receive return ");
    // Serial.println(ret, HEX);
#endif
  }
}

#if SERIAL_DEBUG_MANU_FLG
void serial_manual_display(mbed::CANMessage *msg) {
  Serial.print("left_vert_cyl_pos: ");
  Serial.print(vertical_left_cyl.getLastPosition());
  Serial.print(", right_vert_cyl_pos: ");
  Serial.print(vertical_right_cyl.getLastPosition());
  Serial.print(", left_switch: ");
  Serial.print(left_sw_state);
  Serial.print(", right_switch: ");
  Serial.print(right_sw_state);
  Serial.print(", left_btt: ");
  Serial.print(left_btt_state);
  Serial.print(", right_btt: ");
  Serial.print(right_btt_state);
  Serial.print(", left_encoder: ");
  Serial.print(msg->data[LEFT_ENCODER_IDX]);
  Serial.print(", right_encoder: ");
  Serial.print(msg->data[RIGHT_ENCODER_IDX]);
  Serial.print(", vertical left position: ");
  Serial.print(vertical_left_position_ss.get_raw_position());
  Serial.print(", vertical right position: ");
  Serial.print(vertical_right_position_ss.get_raw_position());
  Serial.print(", bin: ");
  Serial.print(msg->data[BINARY_COMPONENTS_IDX], BIN);

  Serial.print("\n");
}
#endif

#if SERIAL_DEBUG_AUTO_FLG
void serial_auto_display(mbed::CANMessage *msg) {
  Serial.print("Auto!\t ID: ");
  Serial.print(msg->id);
  Serial.print(":");
  for (int i = 0; i < msg->len; i++) {
    Serial.print("\t");
    Serial.print(msg->data[i]);
  }
  Serial.print("\n\r");
}
#endif

#if SERIAL_DEBUG_SEND_FLG
void serial_publish_display(mbed::CANMessage *msg) {
  Serial.print("Publish! can_id: ");
  Serial.print(msg->id, HEX);  // print ID
  Serial.print("\t, length: ");
  Serial.print(msg->len, HEX);  // print DLC
  Serial.print(", \t");

  for (int i = 0; i < msg->len; i++) {  // print the data
    Serial.print(msg->data[i]);
    Serial.print("\t");
  }
  Serial.print("\r\n");
}
#endif

void CANCallback() {
  twai_message_t can_receive_msg;
  int16_t cmd_val;

  int ret = twai_receive(&can_receive_msg, 0);
  if (ret == ESP_OK) {
    switch (can_receive_msg.identifier) {
      case CAN_JOYSTICK_ID_DEC:
        joystick.parseCanJoystickMessage(can_receive_msg.data);
        joystick.controlCylinders();
        // joystick.printJoystickData();

        break;
      case CAN_LEFT_CAM_ID:
        float left_cmd_val;
        cmd_val = can_receive_msg.data[CMD_IDX_UPPER] << 8 |
                  can_receive_msg.data[CMD_IDX_LOWER];
        left_cmd_val = static_cast<float>(cmd_val);
        joystick.setLeftCmdVal(left_cmd_val);
        joystick.controlCylinders();
        left_frame_count = can_receive_msg.data[FRM_CNT_IDX];

#if SERIAL_DEBUG_AUTO_FLG
        serial_auto_display(&can_receive_msg);
#endif
        break;
      case CAN_RIGHT_CAM_ID:
        float right_cmd_val;
        cmd_val = can_receive_msg.data[CMD_IDX_UPPER] << 8 |
                  can_receive_msg.data[CMD_IDX_LOWER];
        right_cmd_val = static_cast<float>(cmd_val);
        joystick.setRightCmdVal(right_cmd_val);
        joystick.controlCylinders();
        right_frame_count = can_receive_msg.data[FRM_CNT_IDX];

#if SERIAL_DEBUG_AUTO_FLG
        serial_auto_display(&can_receive_msg);
#endif
        break;
#if SERIAL_DEBUG_AUTO_FLG
        serial_auto_display(&can_receive_msg);
#endif
        break;
      default:
#if SERIAL_DEBUG_SEND_FLG or SERIAL_DEBUG_AUTO_FLG or SERIAL_DEBUG_MANU_FLG or \
    SERIAL_GENERAL
        Serial.println(can_receive_msg.identifier);
#endif
        break;
    }
  } else {
#if SERIAL_DEBUG_SEND_FLG or SERIAL_DEBUG_AUTO_FLG or SERIAL_DEBUG_MANU_FLG or \
    SERIAL_GENERAL
    // Serial.print("twai_receive return ");
    // Serial.println(ret, HEX);
#endif
  }
}

// For publishing can messages

CanMessage canMessages[] = {{can_log_publish, CAN_LOGGING_MS, 0},
                            {can_position_publish, CAN_POSITION_MS, 0},
                            {can_celery_stat_publish, CAN_CELERY_STAT_MS, 0},
                            {can_mode_publish, CAN_MODE_MS, 0}};

void setup() {
  setupSerial();
  /*----------------------INITIALIZE SPI
   * PERIPHERAL------------------------------*/
  /*----------------------DO NOT
   * REMOVE------------------------------------------*/
  initSPI();

  /*----------------------DO NOT
   * REMOVE------------------------------------------*/
  /*----------------------INITIALIZE SPI
   * PERIPHERAL------------------------------*/

  initFourTwenty();
  initRelays();
  CANinit();

  left_frame_count = 0;
  right_frame_count = 0;

  left_btt_state = false;
  right_btt_state = false;

  initServo();
}

void loop() {
  for (int i = 0; i < sizeof(canMessages) / sizeof(canMessages[0]); i++) {
    publishIfTimeElapsed(canMessages[i]);
  }

  CANCallback();
#if SERIAL_DEBUG_SEND_FLG
  Serial.print("Left cylinder spool position: ");
  Serial.print(vertical_left_cyl.getLastPosition());
  Serial.print("; Right cylinder spool position: ");
  Serial.println(vertical_right_cyl.getLastPosition());
#endif
}

void initSPI() {
  // used by fourTwenty and microSD
  hspi.begin(SPI_2_CLOCK, 17, SPI_2_MOSI);  //  CLK=18  MISO=19  MOSI=23
  fspi.begin(SPI_3_CLOCK, SPI_3_MISO,
             SPI_3_MOSI);  //  CLK=18  MISO=19  MOSI=23
}

void setupSerial() {
#if SERIAL_DEBUG_SEND_FLG or SERIAL_DEBUG_AUTO_FLG or SERIAL_DEBUG_MANU_FLG or \
    SERIAL_GENERAL
  Serial.begin(115200);
  Serial.println("BEAGLE CONTROL BOARD v2.2.0");
  Serial.println("Date: 2024/02/15");
  Serial.println("LEFT cam ID: 0x014, RIGHT cam ID: 0x015");
  Serial.println("Debug ID: 0x003, Position sensors ID: 0x004");
#endif
}
