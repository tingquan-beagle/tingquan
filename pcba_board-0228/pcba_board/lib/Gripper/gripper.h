#pragma once

#include "cylinder.h"

#define SEQ_START 0
#define SEQ_TRACKING 1
#define SEQ_CUTTING 2
#define SEQ_COMPLETE 3
#define SEQ_STOPPED 4
#define STARTUP_POS 100

#define DELAY_ROTATE_BACK 200
#define DELAY_GRAB 700
#define DELAY_CLOSE 400
#define DELAY_RETRACT 300
#define DELAY_ROTATE 1000
#define DELAY_RELEASE 200
#define DELAY_RESET 500
#define DELAY_COMPLETE 1000

enum GripperStates {
  STANDBY = 0,
  CONTROL_VERTICAL_HEIGHT = 1,
  ROTATE_GRIPPER_BACK = 2,
  NC_AND_EXTEND = 3,
  CLOSE_GRIPPER = 4,
  RETRACT = 5,
  ROTATE_GRIPPER = 6,
  OPEN_GRIPPER = 7,
  RESET_POS = 8,
  RESET_SYS = 9
};

class Gripper {
private:
  uint8_t pin_gripper_open;
  uint8_t pin_gripper_close;
  uint8_t pin_rotation;
  CylinderPneumatic *ptr_hor_cyl;
  CylinderPneumatic *ptr_ver_cyl;

  unsigned long timestamp;
  int state;
  bool gripper_mode;
  int original_pos;
  int trig;
  float ver_cmd_val;

public:
  int seqRet;

  Gripper(uint8_t pin_open, uint8_t pin_close, uint8_t pin_rotate, CylinderPneumatic *ptr_hor, CylinderPneumatic *ptr_ver);
  void init();
  void gripper(int action);
  int seq();

  // Getters for internals of gripper
  int getSeqTimestamp();
  int getGripperState();
  int getOriginalPosition();
  int getTrigVal();
  float getVerCmdVal();

  // Setters for internals of gripper
  void setGripperState(int new_state);
  void setOriginalPosition(int pos_val);
  void setTrigValue(int trig_val);
  void setVerCmdVal(float ver_val);
  void setGripperMode(float mode_val);
};
