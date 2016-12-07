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
// #include <TimerThree.h>        //set PWM freq  http://forum.arduino.cc/index.php?topic=117425.0


// #include <math.h>

#if defined(__SAM3X8E__)
#undef __FlashStringHelper::F(string_literal)
#define F(string_literal) string_literal
#endif


// User Defined Function Declarations
void testArc(uint16_t color);
void lcd_write(String str, uint16_t row);
float IR1_distance(int adc_val);


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

#define __IR1_COEFF 0.000054561
#define __IR1_EXP -1.3986

// Global Variables
int adc_read = 0;
float pot_read = 0;
float pot_avg = 0;   //Moving average

long sensor_sum = 0;
int loop_counter = 0;
int loop_counter2 = 0;
int sensor_mean = 0;
int sensor_mean_prev = 1;
bool flag = false;

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

  Serial.println(F("Let's take some data"));

  lcd_write("ADC:",1);
  lcd_write("Distance", 3);

  Serial.println(F("...soon."));

}



// ARDUINO MAIN LOOP FUNCTION
void loop(void) {

  //READ JOYSTICK POTENTIOMETER
  // adc_read = analogRead(A1); //0-1023 maps to 0-5V. Therefore, 0-675 maps to 0-3.3V
  // pot_read = __THROTTLE_MAX-(((float)(adc_read)/675.0) * __THROTTLE_MAX);
  // pot_avg = (pot_avg + 4*pot_read)/5; //Filter using moving average



  // if (!flag){
  //   adc_read = analogRead(__POT); //0-1023 maps to 0-5V. Therefore, 0-675 maps to 0-3.3V
  //   pot_read = __THROTTLE_MAX-(((float)(adc_read)/675.0) * __THROTTLE_MAX);
  //   lcd_write(String(pot_read), 2);
  //   if (pot_read >(__THROTTLE_MAX*0.75)){
  //     flag = true;
  //   }
  //   delay(250);
  // }  else {
    adc_read = analogRead(__IR1);
    sensor_sum+=adc_read;
    loop_counter++;
    if (loop_counter >= 500) {
      // loop_counter2++;
      sensor_mean = sensor_sum/loop_counter;
      Serial.println(String(String(sensor_mean) + " " + String(sensor_mean_prev)));
      Serial.println(String(abs(sensor_mean_prev-sensor_mean)));
      Serial.println(String(((float)(abs(sensor_mean_prev-sensor_mean))/(float)(sensor_mean_prev)
    )));
      if ((((float)(abs(sensor_mean_prev-sensor_mean))/(float)(sensor_mean_prev)) < 0.01 ) && (sensor_mean > 100)){
        // Serial.println(String( String(loop_counter2)+" " + String(sensor_mean) ));
        lcd_write(String(sensor_mean),2);
        lcd_write(String(IR1_distance(sensor_mean)),4);
        loop_counter = 0;
        sensor_sum = 0;
        // flag = false;
      // }
      } else{
        lcd_write("O.O.R.",2);//"out of range"
      }
      sensor_mean_prev = sensor_mean;
  }

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

float IR1_distance(int adc_val){
  //Given input value from 0-1024, returns corresponding distance measurement in inches
  return pow(__IR1_COEFF*(float)(adc_val),__IR1_EXP)* 0.0393701; //converts mm to in
}
