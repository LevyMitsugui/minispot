#ifndef STSCTRL_H
#define STSCTRL_H

#include <SCServo.h>
#include <math.h>

extern SCSCL sc;
extern float ServoDigitalRange_SC;
extern float ServoAngleRange_SC;
extern float ServoDigitalMiddle_SC;
extern const int ServoInitACC_SC;
extern const int ServoMaxSpeed_SC;
extern const int MaxSpeed_X_SC;
extern const int ServoInitSpeed_SC;
extern int MAX_MIN_OFFSET;

extern SMS_STS st;
extern float ServoDigitalRange_ST;
extern float ServoAngleRange_ST;
extern float ServoDigitalMiddle_ST;
extern const int ServoInitACC_ST;
extern const int ServoMaxSpeed_ST;
extern const int MaxSpeed_X_ST;
extern const int ServoInitSpeed_ST;

extern const int S_RXD;
extern const int S_TXD;
extern const int S_SCL;
extern const int S_SDA;
extern int MAX_ID;

extern bool serialFeedback;
extern byte ID_List[253];
extern bool Torque_List[253];
extern int ServoType[253];

extern s16 loadRead[253];
extern s16 speedRead[253];
extern byte voltageRead[253];
extern int currentRead[253];
extern s16 posRead[253];
extern s16 modeRead[253];
extern s16 temperRead[253];

extern byte listID[253];
extern byte searchNum;
extern bool searchedStatus;
extern bool searchFinished;
extern bool searchCmd;
extern byte activeNumInList;
extern s16 activeServoSpeed;
extern byte servotoSet;

extern float linkageBuffer[50];

extern int usbRead;
extern int stsRead;

void getFeedBack(byte servoID);
void getLoadFeedback(byte servoID);
void getPositionFeedback(byte servoID);
void servoInit();
void setMiddle(byte InputID);
void setMode(byte InputID, byte InputMode);
void setID(byte ID_select, byte ID_set);
void servoStop(byte servoID);
void servoTorque(byte servoID, u8 enableCMD);

#endif