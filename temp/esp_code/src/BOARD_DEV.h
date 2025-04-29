#include <Wire.h>
TaskHandle_t ScreenUpdateHandle;
TaskHandle_t ClientCmdHandle;

// SSD1306: 0x3C
#include <Adafruit_SSD1306.h>
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels, 32 as default.
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);


void InitScreen(){
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
  }
  display.clearDisplay();
  display.display();
}


void screenUpdate(){
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);
  // Row1.
  // Row2.
  // Row3.
  display.print(F("MODE:"));
  display.print(float(voltageRead[1])/10);
  if(searchNum){
    display.print(F("N:"));display.print(searchNum);display.print(F(" ID:"));display.print(listID[activeNumInList]);
    display.print(F("M"));display.print(modeRead[listID[activeNumInList]]);display.print(F("T"));display.print(ServoType[listID[activeNumInList]]);
    display.print(F(" P"));display.println(posRead[listID[activeNumInList]]);
  }

  // Row4.
 
  display.println(F("No servo detected."));
  display.display();
}


void pingAll(bool searchCommand){
  if(searchCommand){
    RGBcolor(0, 255, 64);
    searchNum = 0;
    searchedStatus = true;
    searchFinished = false;

    int PingStatus;
    for(int i = 0; i <= MAX_ID; i++){
      PingStatus = st.Ping(i);

      display.clearDisplay();
      display.setTextSize(1);
      display.setTextColor(SSD1306_WHITE);
      display.setCursor(0,0);
      display.println(F("Searching Servos..."));
      display.print(F("MAX_ID "));display.print(MAX_ID);
      display.print(F("-Ping:"));display.println(i);
      display.print(F("Detected:"));

      for(int i = 0; i < searchNum; i++){
        display.print(listID[i]);display.print(F(" "));
      }
      display.display();

      if(PingStatus!=-1){
        listID[searchNum] = i;
        ServoType[i] = st.readByte(i, 3);
        Serial.print("----------->");Serial.print(i);Serial.print("<-----------");Serial.println("");
        Serial.print("----------->");Serial.print(ServoType[i]);Serial.print("<-----------");Serial.println("");
        searchNum++;
      }
      // delay(1);
    }

    Serial.println("ID:");
    for(int i = 0; i< searchNum; i++){
      Serial.print(listID[i]);Serial.print(" ");
      Serial.println("");
    }

    Serial.println("Type:");
    for(int i = 0; i< searchNum; i++){
      Serial.print(ServoType[i]);Serial.print(" ");
      Serial.println("");
    }

    searchedStatus = false;
    searchFinished = true;
    searchCmd      = false;
    RGBoff();
  }
}


void boardDevInit(){
    Wire.begin(S_SDA, S_SCL);
    InitScreen();
    InitRGB();
}


//void espNowSendData(){
  // Set values to send
//  myData.ID_send = listID[activeNumInList];
//  myData.POS_send = posRead[listID[activeNumInList]];
//  myData.Spd_send = speedRead[listID[activeNumInList]];
  
  // Send message via ESP-NOW
//  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
   
 // if (result == ESP_OK) {
 //   Serial.println("Sent with success");
 // }
 // else {
  //  Serial.println("Error sending the data");
 // }
 // delay(200);
//}


void InfoUpdateThreading(void *pvParameter){
  while(1){
    if(!SERIAL_FORWARDING && !RAINBOW_STATUS){
      getFeedBack(listID[activeNumInList]);
      screenUpdate();
      pingAll(searchCmd);
    }
    else if(SERIAL_FORWARDING){
      display.clearDisplay();
      display.setTextSize(1);
      display.setTextColor(SSD1306_WHITE);
      display.setCursor(0,0);
      display.println(F(" - - - - - - - -"));
      display.println(F("SERIAL_FORWARDING"));
      display.println(F(" - - - - - - - -"));
      display.display();
      delay(1000);
    }
    else if(RAINBOW_STATUS){
      display.clearDisplay();
      display.setTextSize(3);
      display.setTextColor(SSD1306_WHITE);
      display.setCursor(0,0);
      display.println(F("RAINBOW"));
      display.display();
      rainbow(30);
    }
  }
}

void threadInit(){
  xTaskCreatePinnedToCore(&InfoUpdateThreading, "InfoUpdate", 4000, NULL, 2, &ScreenUpdateHandle, ARDUINO_RUNNING_CORE);
}
