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

// ROS interface libraries
#include <ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Quaternion.h>

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

ros::NodeHandle nh;
std_msgs::String str_msg;
ros::Publisher test_chatter("test_chatter_topic", &str_msg);
char hello[13] = "hello world!";
sensor_msgs::Imu imu_msg;
ros::Publisher imu_pub("imu_topic", &imu_msg);

// ARDUINO MAIN SETUP FUNCTION
void setup() {
  // Serial.begin(9600);
  tft.begin();
  testArc(GREEN);
  tft.clearScreen();
  // tft.setCursor(0, 0);
  tft.setTextColor(WHITE);
  tft.setTextSize(2);
  // tft.print("Hello World!");

  // initialize ROS node handle, publishers, subscribers
  nh.initNode();
  nh.advertise(test_chatter);
  nh.advertise(imu_pub);

  // Serial.println(F("Let's test this IMU!"));
  // Serial.println(F("...soon."));

  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    // Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    lcd_write(String("IMU err"), 1);
    while(1);
  }
  delay(1000);
  bno.setExtCrystalUse(true);

  // lcd_write("Yaw_x:",1);
  // lcd_write("Pitch_y", 3);
  // lcd_write("Roll_z",5);
}



// ARDUINO MAIN LOOP FUNCTION
void loop(void) {

  /* Get a new sensor event */
  sensors_event_t event;
  bno.getEvent(&event);

  /* Display the floating point data */
  // lcd_write(String(event.orientation.x),2);
  // lcd_write(String(event.orientation.y),4);
  // lcd_write(String(event.orientation.z),6);

  imu::Quaternion quat = bno.getQuat();
  imu_msg.orientation.x = quat.x();
  imu_msg.orientation.y = quat.y();
  imu_msg.orientation.z = quat.z();
  imu_msg.orientation.w = quat.w();

  str_msg.data = hello;

  imu_pub.publish ( &imu_msg );
  test_chatter.publish( &str_msg );
  nh.spinOnce();
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
