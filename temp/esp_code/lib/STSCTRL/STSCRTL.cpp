#include "STSCTRL.h"

SCSCL sc;
float ServoDigitalRange_SC = 1023.0;
float ServoAngleRange_SC = 210.0;
float ServoDigitalMiddle_SC = 511.0;
const int ServoInitACC_SC = 0;
const int ServoMaxSpeed_SC = 1500;
const int MaxSpeed_X_SC = 1500;
const int ServoInitSpeed_SC = 800;
int MAX_MIN_OFFSET = 30;

SMS_STS st;
float ServoDigitalRange_ST = 4095.0;
float ServoAngleRange_ST = 360.0;
float ServoDigitalMiddle_ST = 2047.0;
const int ServoInitACC_ST = 100;
const int ServoMaxSpeed_ST = 4000;
const int MaxSpeed_X_ST = 4000;
const int ServoInitSpeed_ST = 2000;

const int S_RXD = 18;
const int S_TXD = 19;
const int S_SCL = 22;
const int S_SDA = 21;
int MAX_ID = 20;

bool serialFeedback = true;

byte ID_List[253];
bool Torque_List[253];
int ServoType[253];

s16 loadRead[253];
s16 speedRead[253];
byte voltageRead[253];
int currentRead[253];
s16 posRead[253];
s16 modeRead[253];
s16 temperRead[253];

byte listID[253];
byte searchNum = 0;
bool searchedStatus = false;
bool searchFinished = false;
bool searchCmd = false;
byte activeNumInList = 0;
s16 activeServoSpeed = 100;
byte servotoSet = 0;

float linkageBuffer[50];

int usbRead;
int stsRead;

void getFeedBack(byte servoID) {
    if (ServoType[servoID] == 9) {
        if (st.FeedBack(servoID) != -1) {
            posRead[servoID] = st.ReadPos(-1);
            speedRead[servoID] = st.ReadSpeed(-1);
            loadRead[servoID] = st.ReadLoad(-1);
            voltageRead[servoID] = st.ReadVoltage(-1);
            currentRead[servoID] = st.ReadCurrent(-1);
            temperRead[servoID] = st.ReadTemper(-1);
            modeRead[servoID] = st.ReadMode(servoID);
        } else {
            if (serialFeedback) {
                Serial.println("FeedBack err");
            }
        }
    }
}

void getLoadFeedback(byte servoID) {
    if (ServoType[servoID] == 9) {
        if(st.FeedBack(servoID) != -1){
            loadRead[servoID] = st.ReadLoad(-1);
        }
        else{
            if(serialFeedback){
                Serial.println("FeedBack err");
                Serial.println(servoID);
            }
        }
    }
}

void getPositionFeedback(byte servoID){
    if (ServoType[servoID] == 9) {
        if(st.FeedBack(servoID) != -1){
            posRead[servoID] = st.ReadPos(-1);
        }
        else{
            if(serialFeedback){
                Serial.println("FeedBack err");
                //Serial.println(servoID);
            }
        }
    }
}
void servoInit() {
    Serial1.begin(1000000, SERIAL_8N1, S_RXD, S_TXD);
    sc.pSerial = &Serial1;
    st.pSerial = &Serial1;
    while (!Serial1) {}

    for (int i = 0; i < MAX_ID; i++) {
        Torque_List[i] = true;
        ServoType[i] = -1;
    }
}

void setMiddle(byte InputID) {
    if (ServoType[InputID] == 9) {
        st.CalibrationOfs(InputID);
    }
}

void setMode(byte InputID, byte InputMode) {
    // Your code for setting mode
  if(InputMode == 0){
    if(ServoType[InputID] == 9){
      st.unLockEprom(InputID);
      st.writeWord(InputID, 11, 4095);
      st.writeByte(InputID, SMS_STS_MODE, InputMode);
      st.LockEprom(InputID);
    }
    else if(ServoType[InputID] == 5){
      sc.unLockEprom(InputID);
      sc.writeWord(InputID, SCSCL_MIN_ANGLE_LIMIT_L, 20);
      sc.writeWord(InputID, SCSCL_MAX_ANGLE_LIMIT_L, 1003);
      sc.LockEprom(InputID);
    }
  }
  else if(InputMode == 3){
    if(ServoType[InputID] == 9){
      st.unLockEprom(InputID);
      st.writeByte(InputID, SMS_STS_MODE, InputMode);
      st.writeWord(InputID, 11, 0);
      st.LockEprom(InputID);
    }
    else if(ServoType[InputID] == 5){
      sc.unLockEprom(InputID);
      sc.writeWord(InputID, SCSCL_MIN_ANGLE_LIMIT_L, 0);
      sc.writeWord(InputID, SCSCL_MAX_ANGLE_LIMIT_L, 0);
      sc.LockEprom(InputID);
    }
  }
}

void setID(byte ID_select, byte ID_set) {
    // Your code for setting ID
    if(ID_set > MAX_ID){MAX_ID = ID_set;}

  if(ServoType[ID_select] == 9){
    st.unLockEprom(ID_select);
    st.writeByte(ID_select, SMS_STS_ID, ID_set);
    st.LockEprom(ID_set);
  }
  else if(ServoType[ID_select] == 5){
    sc.unLockEprom(ID_select);
    sc.writeByte(ID_select, SCSCL_ID, ID_set);
    sc.LockEprom(ID_set);
  }
}

void servoStop(byte servoID) {
    // Your code for stopping servo
    if(ServoType[servoID] == 9){
    st.EnableTorque(servoID, 0);
    delay(10);
    st.EnableTorque(servoID, 1);
  }
  else if(ServoType[servoID] == 5){
    sc.EnableTorque(servoID, 0);
    delay(10);
    sc.EnableTorque(servoID, 1);
  }
  
}

void servoTorque(byte servoID, u8 enableCMD) {
    // Your code for enabling/disabling torque
  if(ServoType[servoID] == 9){
    st.EnableTorque(servoID, enableCMD);
  }
  else if(ServoType[servoID] == 5){
    sc.EnableTorque(servoID, enableCMD);
  }
}