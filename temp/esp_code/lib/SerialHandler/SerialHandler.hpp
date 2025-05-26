#include <Arduino.h>

#define COMMAND_BUFFER_SIZE 256

typedef struct PI_COMMAND
{
  bool new_command;
  int command;

  char package[COMMAND_BUFFER_SIZE];
  int package_len;

} PI_COMMAND;

typedef struct SERVO_FRAME
{
  float pos[12];
  float speed[12];
} SERVO_FRAME;

typedef struct OFFSET_LEAN_FRAME
{
  float rpy[3]; // Roll, Pitch, Yaw
  float xyz[3]; // X, Y, Z
  float speed;
} OFFSET_LEAN_FRAME;

class SerialHandler
{
public:
    //virtual void handleSerialEvent() = 0; //what is this?
    //using Callback = void (*)(const char *);
    bool callback_enabled = false;

    SerialHandler();
    void HandleSerialEvent(char * inputBuffer, int & bufferPos, PI_COMMAND & pi_command, OFFSET_LEAN_FRAME & offset_lean_frame, SERVO_FRAME & servo_frame);

private:
    void processPiCommand(const char *cmd, PI_COMMAND & pi_command);
    int parseServoFrame(char *buf, SERVO_FRAME & servo_frame);
    int parseOffsetLean(char *buf, OFFSET_LEAN_FRAME & offset_lean_frame);
};