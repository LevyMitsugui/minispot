#include <BMI160Gen.h>            //https://github.com/hanyazou/BMI160-Arduino
#include <Wire.h>
#include <Arduino.h>

#define SDA_PIN 0
#define SCL_PIN 1

#define ACCELEROMETER_SCALE 0.000598
 
// I2C Configuration for ESP32
const int i2c_addr = 0x68;  // Default I2C address for BMI160

long pico_time = 0;
long prev_time = 0;
long cycle = 10;	// cycle period in milliseconds

long prev_time_debug = 0;
long cycle_debug = 80;
bool error_state = false;

bool led_state = false;
int led_count = 0;


int gx, gy, gz; // Raw gyroscope values
int ax, ay, az; // Raw accelerometer values

float accel_mps2_x = 0, accel_mps2_y = 0, accel_mps2_z = 0;


void morse_serial_down();
void morse_error();

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);

  Serial.begin(230400); // TODO: tone it down to 115200, remember to do the same in .ini and the python script
  while (!Serial){
    morse_serial_down();
  }
  for(int i=0; i<2; i++){
    digitalWrite(LED_BUILTIN, HIGH);
    delay(80);
    digitalWrite(LED_BUILTIN, LOW);
    delay(80);
    }
  
  pinMode(25, OUTPUT); // Onboard LED

  Wire.setSDA(SDA_PIN);
  Wire.setSCL(SCL_PIN);
  Wire.begin();
 

  if (!BMI160.begin(BMI160GenClass::I2C_MODE, i2c_addr)) {
    Serial.println("BMI160 initialization failed!");
    while (1); // Halt if initialization fails
  }
 
  Serial.println("BMI160 initialized successfully in I2C mode!");
}
 
void loop() {
  while(!Serial){
    morse_serial_down();
  }

  if(error_state){
    morse_error();
  }

  pico_time = millis();
  if ((pico_time - prev_time) > cycle) {
    prev_time = pico_time;
  
    BMI160.readGyro(gx, gy, gz); 
    BMI160.readAccelerometer(ax, ay, az);

    accel_mps2_x = ax * 0.000061035f * 9.80665f;
    accel_mps2_y = ay * 0.000061035f * 9.80665f;
    accel_mps2_z = az * 0.000061035f * 9.80665f;
  
    String topic = "pico/accel";
    String line = topic + ":" + String(pico_time) + "," + String(accel_mps2_x, 3) + "," + String(accel_mps2_y, 3) + "," + String(accel_mps2_z, 3) + "\n";
    Serial.write(line.c_str(), line.length());

    // Serial.print("pico/accel:");
    // Serial.print(((float)pico_time)/1000, 3);
    // Serial.print(",");
    // Serial.print(accel_mps2_x );
    // Serial.print(",");
    // Serial.print(accel_mps2_y);
    // Serial.print(",");
    // Serial.print(accel_mps2_z);
    // Serial.print("\n");

    
    // Serial.print(gx);
    // Serial.print("\t");
    // Serial.print(gy);
    // Serial.print("\t");
    // Serial.print(gz);
    // Serial.print("\t");
  }

  if ((pico_time - prev_time_debug) > cycle_debug) {//TODO fix this
    prev_time_debug = pico_time;
    led_state = !led_state;
    digitalWrite(25, led_state);
    if(led_state)led_count++;
    if (led_count % 2 == 0){
      cycle_debug = 2000;
    } else {
      cycle_debug = 80;
    }
  }
}

void morse_serial_down(){
  for(int i=0; i<3; i++){
    digitalWrite(LED_BUILTIN, HIGH);
    delay(150);
    digitalWrite(LED_BUILTIN, LOW);
    delay(150);
    }
    delay (750);
}

void morse_error(){
  digitalWrite(LED_BUILTIN, HIGH);
  delay(150);
  digitalWrite(LED_BUILTIN, LOW);
  delay(750);
  for(int i=0; i<2; i++){
    digitalWrite(LED_BUILTIN, HIGH);
    delay(150);
    digitalWrite(LED_BUILTIN, LOW);
    delay(150);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(500);
    digitalWrite(LED_BUILTIN, LOW);
    delay(150);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(150);
    digitalWrite(LED_BUILTIN, LOW);
    delay(750);
  }
}