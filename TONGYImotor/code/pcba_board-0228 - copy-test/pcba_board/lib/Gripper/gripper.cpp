#include "gripper.h"
#include "relay.h"
#include "Servo.h"

Gripper::Gripper(uint8_t pin_open, uint8_t pin_close, uint8_t pin_rotate, CylinderPneumatic *ptr_hor, CylinderPneumatic *ptr_ver) {
  pin_gripper_open = pin_open;
  pin_gripper_close = pin_close;
  pin_rotation = pin_rotate;
  ptr_hor_cyl = ptr_hor;
  ptr_ver_cyl = ptr_ver;
  timestamp = 0L;
  state = 0;
  original_pos = 0;
  trig = 0;
  ver_cmd_val = 0.0;
  seqRet = SEQ_STOPPED;
}

void Gripper::init() {
}

void Gripper::gripper(int action){
    switch (action) {
        case 0: // Gripper closes
            digitalWrite(pin_gripper_open, LOW);
            digitalWrite(pin_gripper_close, HIGH);
            break;
        case 1: // Gripper opens
            digitalWrite(pin_gripper_close, LOW);
            digitalWrite(pin_gripper_open, HIGH);
            break;
        case 2: // Gripper returns to neutral
            digitalWrite(pin_gripper_open, LOW);
            digitalWrite(pin_gripper_close, LOW);
            break;
    }
}
  
int Gripper::seq(){
    switch (state){

        // standby
        case STANDBY:
        gripper(1); // open gripper
        digitalWrite(pin_rotation, HIGH); // rotate gripper for camera view
        timestamp = millis(); // keep resetting timestamp for consecutive timers
        if (trig == 1 && gripper_mode == AUTO) {
            original_pos = ptr_ver_cyl->getLastPosition(); // record original position
            ptr_ver_cyl->setBasePosition(); // set the base position to mimic auto mode
            state = CONTROL_VERTICAL_HEIGHT;
        }
        seqRet = SEQ_START;
        break;

        // start PID control and horizontal extension
        case CONTROL_VERTICAL_HEIGHT:
        ptr_ver_cyl->setPosition(int(ver_cmd_val)); // start vertical tracking of celery
        if (gripper_mode == MANUAL) {
            ptr_ver_cyl->setVal(original_pos); // set back to original recorded position
            ptr_hor_cyl->setVal(0); // retract entirely
            state = STANDBY;
        }
        else if (trig == 2 && gripper_mode == AUTO) { // set by jetson
            state = ROTATE_GRIPPER_BACK; // finish control and close
            timestamp = millis();
        }
        seqRet = SEQ_TRACKING;
        break;

        // rotate gripper back to position
        case ROTATE_GRIPPER_BACK:
        digitalWrite(pin_rotation, LOW); // rotate gripper
        if (millis() - timestamp >= DELAY_ROTATE_BACK){
            state = NC_AND_EXTEND;
            timestamp = millis();
        }
        seqRet = SEQ_CUTTING;
        break;

        // gripper opening and cylinder extending; NC == no control of vertical cylinder
        case NC_AND_EXTEND:
        if (millis() - timestamp >= DELAY_GRAB){
            gripper(1); // open gripper
            ptr_hor_cyl->setVal(255); // full extension
            state = CLOSE_GRIPPER; // while closing, continue extension
            timestamp = millis();
        }
        seqRet = SEQ_CUTTING;
        break;

        // close gripper
        case CLOSE_GRIPPER:
        if(millis() - timestamp >= DELAY_CLOSE) {
            gripper(0); // close gripper
            state = RETRACT; // retract gripper
            timestamp = millis();
        }
        seqRet = SEQ_CUTTING;
        break;

        // retract gripper
        case RETRACT:
        if(millis() - timestamp >= DELAY_RETRACT){
            //gripper(0); // close gripper TODO: TALK WITH MING ABOUT THIS TIMING
            ptr_hor_cyl->setVal(0);
            ptr_ver_cyl->setVal(0);

            state = ROTATE_GRIPPER;
            timestamp = millis();
        }
        seqRet = SEQ_CUTTING;
        break;

        // gripper rotates
        case ROTATE_GRIPPER:
        if (millis() - timestamp >= DELAY_ROTATE){
            digitalWrite(pin_rotation, HIGH); // rotate gripper

            state = OPEN_GRIPPER;
            timestamp = millis();
        }
        seqRet = SEQ_CUTTING;
        break;

        // gripper opens
        case OPEN_GRIPPER:
        if(millis() - timestamp >= DELAY_RELEASE){
            gripper(1); // open

            state = RESET_POS;
            timestamp = millis();
        }  
        seqRet = SEQ_CUTTING;
        break;

        // reset positions
        case RESET_POS:
        gripper(2); // return to neutral
        if(millis() - timestamp >= DELAY_RESET){ 
            state = RESET_SYS;
            timestamp = millis();
        }
        seqRet = SEQ_CUTTING;
        break;

        case RESET_SYS: // reset state and send SEQ_COMPLETE TODO: DELETE STATE
        if (millis() - timestamp >= DELAY_COMPLETE){
            ptr_ver_cyl->setVal(original_pos);
            state = STANDBY;
            seqRet = SEQ_COMPLETE;
        }
        break;

        default:
        seqRet = SEQ_STOPPED;
        break;
    }

    return seqRet;

}

  // Getters for internals of gripper.
int Gripper::getSeqTimestamp() { return timestamp; }

int Gripper::getGripperState() { return state; }

int Gripper::getOriginalPosition() { return original_pos; }

int Gripper::getTrigVal() { return trig; }

float Gripper::getVerCmdVal() { return ver_cmd_val; }

// Setters for internals of gripper.
void Gripper::setGripperState(int new_state) { state = new_state; }

void Gripper::setOriginalPosition(int pos_val) { original_pos = pos_val; }

void Gripper::setTrigValue(int trig_val) { trig = trig_val; }

void Gripper::setVerCmdVal(float ver_val) { ver_cmd_val = ver_val; }

void Gripper::setGripperMode(float mode_val) { gripper_mode = mode_val; }




GripperMotor::GripperMotor(uint8_t pin_open, uint8_t pin_close, uint8_t pin_rotate, uint8_t pin_extend, uint8_t second_cutter) {
  pin_gripper_open = pin_open;
  pin_gripper_close = pin_close;
  pin_rotation = pin_rotate;
  pin_hor_extend = pin_extend;
  pin_second_cutter = second_cutter;
  timestamp = 0L;
  state = 0;
  original_pos = 0;
  trig = 0;
  ver_cmd_val = 0.0;
  seqRet = SEQ_STOPPED;
  servospeed = 0x00007530;// speed is 3000r/min
}

void GripperMotor::init() {
}

void GripperMotor::gripper(int action){
    switch (action) {
        case 0: // Gripper closes
            setRelay(pin_gripper_open, LOW);
            setRelay(pin_gripper_close, HIGH);
            break;
        case 1: // Gripper opens
            setRelay(pin_gripper_close, LOW);
            setRelay(pin_gripper_open, HIGH);
            break;
        case 2: // Gripper returns to neutral
            setRelay(pin_gripper_open, LOW);
            setRelay(pin_gripper_close, LOW);
            break;
    }
}
  
int GripperMotor::seq(){
    switch (state){

        // standby
        case STANDBY:
        gripper(2); // Gripper returns to neutral
        setRelay(pin_rotation,LOW);
        setRelay(pin_hor_extend,LOW);
        setRelay(pin_second_cutter,LOW);
        setServoTarget(0xFFCE0000,servospeed); //if 50-rounds position is zero,-50 is standby position,stroke is 0cm
        // Serial.println("standby");
        seqRet = SEQ_START;
        state = CONTROL_VERTICAL_HEIGHT;
        timestamp = millis();
        break;

        // vertical servo extension
        case CONTROL_VERTICAL_HEIGHT:
          setServoTarget(0xFFE70000,servospeed); // set the stroke is 25cm,command is -25 rounds
        //Serial.println("CONTROL_VERTICAL_HEIGHT"); 
          state = NC_AND_EXTEND; 
          seqRet = SEQ_TRACKING;
          timestamp = millis();
        break;

        // gripper opening and cylinder extending; NC == no control of vertical cylinder
        case NC_AND_EXTEND:
        if (millis() - timestamp >= DELAY_GRAB){
            gripper(1); // open gripper
            // digitalWrite(pin_hor_extend, HIGH);// full extension
            setServoTarget(0xFFEC0000,servospeed); // set the stroke is 30cm,command is -20 rounds
            setRelay(pin_hor_extend,HIGH);
            // Serial.println("gripper opening and cylinder extending");
            state = CLOSE_GRIPPER; // while closing, continue extension
            timestamp = millis();
        }
        seqRet = SEQ_CUTTING;
        break;

        // close gripper
        case CLOSE_GRIPPER:
        if(millis() - timestamp >= DELAY_CLOSE) {
            gripper(0); // close gripper
            // Serial.println("close gripper");
            state = SECOND_CUTTER_EXTEND; 
            timestamp = millis();
        }
        seqRet = SEQ_CUTTING;
        break;

        case SECOND_CUTTER_EXTEND:
        if(millis() - timestamp >= DELAY_SECOND_CUTTER_EXTEND) {
            setRelay( pin_second_cutter, HIGH);
            // Serial.println("second cutter extend");
            state = SECOND_CUTTER_RETRACT; 
            timestamp = millis();
        }
        seqRet = SEQ_CUTTING;
        break;

        case SECOND_CUTTER_RETRACT:
        if(millis() - timestamp >= DELAY_SECOND_CUTTER_RETRACT) {
            setRelay( pin_second_cutter, LOW);
            // Serial.println("second cutter extend");
            state = RETRACT; // retract gripper
            timestamp = millis();
        }
        seqRet = SEQ_CUTTING;
        break;

        // retract gripper
        case RETRACT:
        if(millis() - timestamp >= DELAY_RETRACT){
            // gripper(0); // close gripper TODO: TALK WITH MING ABOUT THIS TIMING
            // digitalWrite(pin_hor_extend, LOW);
            setRelay(pin_hor_extend,LOW);
            // Serial.println("retract gripper");
            state = ROTATE_GRIPPER;
            timestamp = millis();
        }
        seqRet = SEQ_CUTTING;
        break;

        // gripper rotates
        case ROTATE_GRIPPER:
        if (millis() - timestamp >= DELAY_ROTATE){
            // digitalWrite(pin_rotation, HIGH); // rotate gripper
            setRelay(pin_rotation,HIGH);
            // Serial.println("gripper rotates");
            state = OPEN_GRIPPER;
            timestamp = millis();
        }
        seqRet = SEQ_CUTTING;
        break;

        // gripper opens
        case OPEN_GRIPPER:
        if(millis() - timestamp >= DELAY_RELEASE){
            gripper(1); // open
            // Serial.println("gripper opens");
            state = RESET_POS;
            timestamp = millis();
        }  
        seqRet = SEQ_CUTTING;
        break;

        // reset positions
        case RESET_POS:
        if(millis() - timestamp >= DELAY_RESET){ 
            gripper(2); // return to neutral
            // Serial.println("gripper return to neutral");
            state = RESET_SYS;
            timestamp = millis();
        }
        seqRet = SEQ_CUTTING;
        break;

        case RESET_SYS: // reset state and send SEQ_COMPLETE TODO: DELETE STATE
        if (millis() - timestamp >= DELAY_COMPLETE){
            // digitalWrite(pin_rotation, LOW);
            setRelay(pin_rotation,LOW);
            delay(700);
            setServoTarget(0xFFCE0000,servospeed); 
            // Serial.println("RESET_SYS");
            state = STANDBY;
            seqRet = SEQ_COMPLETE;
            delay(1500);
        }
        break;

        default:
        seqRet = SEQ_STOPPED;
        break;
    }

    return seqRet;

}

void GripperMotor::gripperReset(){

    gripper(2); //grippers open
    setRelay(pin_rotation,LOW);
    setRelay(pin_hor_extend,LOW);
    setServoTarget(0xFFCE0000,servospeed);
    state = STANDBY;
    // Serial.println("STANDBY");
}

  // Getters for internals of gripper.
int GripperMotor::getSeqTimestamp() { return timestamp; }

int GripperMotor::getGripperState() { return state; }

// Setters for internals of gripper.
void GripperMotor::setGripperState(int new_state) { state = new_state; }