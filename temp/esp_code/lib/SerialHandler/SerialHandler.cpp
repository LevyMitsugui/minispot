#include "SerialHandler.hpp"

SerialHandler::SerialHandler():
    callback_enabled(false) {}

void SerialHandler::HandleSerialEvent(char * inputBuffer, int & bufferPos, PI_COMMAND & pi_command, OFFSET_LEAN_FRAME & offset_lean_frame, SERVO_FRAME & servo_frame)
{
    if (inputBuffer == NULL  )
    {
        Serial.println("[ERROR] Invalid input buffer");
        return;
    }
    while (Serial.available())
    {
        char c = Serial.read();
        
        Serial.print("Received: ");
        Serial.print(c);
        Serial.print("  bufferPos: ");
        Serial.println(bufferPos);
        
        if (c == '\n' && !callback_enabled)
        {
            inputBuffer[bufferPos] = '\0'; // Null-terminate string
            if (bufferPos > 0)
            {
                if (inputBuffer[0] == '<')
                {
                    processPiCommand(inputBuffer + 1, pi_command);
                }
                else if (inputBuffer[0] == '>')
                { // handle stream mode
                    Serial.println("ESP/STREAM:Stream Control Enabled");
                    callback_enabled = true;
                }
                else if (inputBuffer[0] == '/' && inputBuffer[1] == 'k')
                {
                    ESP.restart();
                } else {
                    Serial.print("[ERROR] Unknown command prefix, function got: ");
                    for(int i = 0; i < bufferPos; i++){
                      Serial.print(inputBuffer[i]);
                    }
                    Serial.println();

                }
            }
            bufferPos = 0; // Reset buffer
        }
        else if (!callback_enabled && (bufferPos < COMMAND_BUFFER_SIZE - 1))
        {
            inputBuffer[bufferPos++] = c;
        }
        // else if (stream_control)
        // {
        //     process_stream_control(c); //TODO run callback
        // }
        else
        {
            Serial.println("[ERROR] Command too long");
            bufferPos = 0;
        }
    }
}

void SerialHandler::processPiCommand(const char *cmd, PI_COMMAND & pi_command){
  strncpy(pi_command.package, cmd, COMMAND_BUFFER_SIZE);
  pi_command.package[COMMAND_BUFFER_SIZE - 1] = '\0';

  char *token = strtok(pi_command.package, ":");
  pi_command.command = atoi(token);
  pi_command.new_command = true;
}
int SerialHandler::parseServoFrame(char *buf, SERVO_FRAME & servo_frame){
  int i = 0;
  int idx = 0;

  char *token = strtok(buf, ",");
  while (token != NULL)
  {
    if (i % 2 == 0)
    {
      servo_frame.pos[idx] = atof(token);
    }
    else
    {
      servo_frame.speed[idx] = atof(token);
      idx++;
    }
    token = strtok(NULL, ",");
    i++;
  }
  if (i < 24)
  {
    return -1;
    Serial.println("[ERROR] Invalid servo frame");
  }
  return 0;
}
int SerialHandler::parseOffsetLean(char * buf, OFFSET_LEAN_FRAME & offset_lean_frame){
  char *token = strtok(buf, ",");
  for (int i = 0; i < 3; i++)
  {
    // Serial.println(token);
    offset_lean_frame.xyz[i] = atof(token);
    token = strtok(NULL, ",");
    // Serial.println(token);
    offset_lean_frame.rpy[i] = atof(token);
    token = strtok(NULL, ",");

    if (i == 2)
    {
      offset_lean_frame.speed = atof(token);
    }
  }
  return 0;
}