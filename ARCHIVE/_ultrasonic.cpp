/*
    ALAN HONG
    POWERBOARD PCB CONTROL CODE
    This is a test file for the ultrasonic ping sensor.
*/

//LIBRARIES
#include "Arduino.h"
#include <string.h>
#include <SPI.h>
#include <TFT_ST7735.h> //LCD screen
// #include <TimerThree.h>        //set PWM freq  http://forum.arduino.cc/index.php?topic=117425.0
#include <NewPing.h> //Ultrasonic sensor

// #include <math.h>

#if defined(__SAM3X8E__)
#undef __FlashStringHelper::F(string_literal)
#define F(string_literal) string_literal
#endif


// User Defined Function Declarations
void testArc(uint16_t color);
void lcd_write(String str, uint16_t row);
void echoCheck();

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

#define __PING_PIN     11  // ultrasonic sensor digital pin
#define __MAX_DISTANCE 300 // Max desired distance measurement (in centimeters). Maximum sensor distance is rated at 300cm.

// Global Variables
int adc_read = 0;
float pot_read = 0;
float pot_avg = 0;   //Moving average

NewPing sonar(__PING_PIN, __PING_PIN, __MAX_DISTANCE);
unsigned int pingSpeed = 100; // How frequently are we going to send out a ping (in milliseconds). 50ms would be 20 times a second.
unsigned long pingTimer;     // Holds the next ping time.

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

  lcd_write("Distance:",1);

  Serial.println(F("...soon."));

  pingTimer = millis();
}



// ARDUINO MAIN LOOP FUNCTION
void loop(void) {

  //READ JOYSTICK POTENTIOMETER
  // adc_read = analogRead(A1); //0-1023 maps to 0-5V. Therefore, 0-675 maps to 0-3.3V
  // pot_read = __THROTTLE_MAX-(((float)(adc_read)/675.0) * __THROTTLE_MAX);
  // pot_avg = (pot_avg + 4*pot_read)/5; //Filter using moving average

  if (millis() >= pingTimer) {   // pingSpeed milliseconds since last ping, do another ping.
    pingTimer += pingSpeed;      // Set the next ping time.
    sonar.ping_timer(echoCheck); // Send out the ping, calls "echoCheck" function every 24uS where you can check the ping status.
  }

  // lcd_write(String(sonar.ping_in()),2);
  // delay(500);
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

void echoCheck() { // Timer2 interrupt calls this function every 24uS where you can check the ping status.
  // Don't do anything here!
  if (sonar.check_timer()) { // This is how you check to see if the ping was received.
    // Here's where you can add code.
    // lcd_write(String(sonar.ping_result / US_ROUNDTRIP_IN),2); // Ping returned, uS result in ping_result, convert to cm with US_ROUNDTRIP_CM.
    Serial.println(String(sonar.ping_result / US_ROUNDTRIP_IN));
  }
  // Don't do anything here!
}
