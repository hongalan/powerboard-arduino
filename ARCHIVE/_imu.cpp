/*
    ALAN HONG
    POWERBOARD PCB CONTROL CODE
    This is a test file for the IR range sensor.
*/

//LIBRARIES
#include "Arduino.h"
#include <string.h>
#include <SPI.h>
#include <TFT_ST7735.h> //LCD screen
#include <Wire.h>
// #include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

// #include <math.h>

#if defined(__SAM3X8E__)
#undef __FlashStringHelper::F(string_literal)
#define F(string_literal) string_literal
#endif


// User Defined Function Declarations
void testArc(uint16_t color);
void lcd_write(String str, uint16_t row);

// Color definitions
#define	BLUE    0x001F
#define	RED     0xF800
#define	GREEN   0x07E0
#define CYAN    0x07FF
#define MAGENTA 0xF81F
#define YELLOW  0xFFE0

// Pin Mapping
#define __CS 	49       //SPI Chip Select
#define __DC 	48       //SPI Data
#define __RST 	255    //SPI Reset (not used)
// #define __ESC_SIG 2    //ESC signal PWM pin
#define __THROTTLE_MAX 1000
#define __IR1 A4
#define __POT A1


// Global Variables
int adc_read = 0;
float pot_read = 0;
float pot_avg = 0;   //Moving average

Adafruit_BNO055 bno = Adafruit_BNO055(55);
float acc_curr_x = 0;
float acc_prev_x = 0;
float vel_integral_x = 0;
float pos_integral_x = 0;

long time_prev = 0;
float time_diff = 0; //time elapsed in seconds


TFT_ST7735 tft = TFT_ST7735(__CS, __DC, __RST);

// ARDUINO MAIN SETUP FUNCTION
void setup() {
  Serial.begin(9600);
  tft.begin();

  testArc(GREEN);
  tft.clearScreen();
  // tft.setCursor(0, 0);
  tft.setTextColor(WHITE);
  tft.setTextSize(2);
  // tft.print("Hello World!");

  Serial.println(F("Let's test this IMU!"));

  // lcd_write("Yaw_x:",1);
  // lcd_write("Pitch_y", 3);
  // lcd_write("Roll_z",5);

  lcd_write("Acc_x:",1);
  lcd_write("Vel_x", 3);
  lcd_write("Pos_x",5);

  Serial.println(F("...soon."));

  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  delay(1000);
  bno.setExtCrystalUse(true);
  imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  acc_prev_x = accel.x()*39.3701;
  acc_curr_x = (acc_prev_x + 3*accel.x()*39.3701)*0.25;
  time_prev = micros();
}



// ARDUINO MAIN LOOP FUNCTION
void loop(void) {

  //READ JOYSTICK POTENTIOMETER
  // adc_read = analogRead(A1); //0-1023 maps to 0-5V. Therefore, 0-675 maps to 0-3.3V
  // pot_read = __THROTTLE_MAX-(((float)(adc_read)/675.0) * __THROTTLE_MAX);
  // pot_avg = (pot_avg + 4*pot_read)/5; //Filter using moving average

  /* Get a new sensor event */
  sensors_event_t event;
  bno.getEvent(&event);

  /* Display the floating point data */
  // lcd_write(String(event.orientation.x),2);
  // lcd_write(String(event.orientation.y),4);
  // lcd_write(String(event.orientation.z),6);
  time_diff = (micros() - time_prev)*0.000001; //convert microseconds to seconds
  time_prev = micros();
  imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  acc_curr_x = (acc_prev_x + 3*accel.x()*39.3701)*0.25;
  vel_integral_x += (acc_prev_x + 0.5*((acc_curr_x-acc_prev_x)))*time_diff; //convert to inches
  pos_integral_x += (0.5*acc_prev_x + 0.1667*(acc_curr_x-acc_prev_x))*pow(time_diff,2);
  lcd_write(String(accel.x()*39.3701),2);
  lcd_write(String(vel_integral_x),4);
  lcd_write(String(pos_integral_x),6);
  acc_prev_x = accel.x()*39.3701;
  delay(100);

}


//USER DEFINED FUNCTIONS
void testArc(uint16_t color) {
  // unsigned long start;
  uint16_t      i,
                cx = tft.width()  / 2,
                cy = tft.height() / 2;
  tft.clearScreen();
  // start = micros();
  for (i = 0; i < 360; i += 5) {
    tft.drawArc(cx, cy, 40, 2, 0, i, color);
  }
  // return micros() - start;
}

void lcd_write(String str, uint16_t row){
  //Using Arduino String class
  tft.fillRect(0, (row-1)*20, 128, 20, BLACK);
  tft.setCursor(0, (row-1)*20);
  tft.print(str);
}
