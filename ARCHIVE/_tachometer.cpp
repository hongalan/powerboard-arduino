/*
    ALAN HONG
    TEST SCRIPT FOR DIY TACHOMETER

    Hardware description:
    Attach strip of tape to motor such that the tape has a tab extending out.
    Arrange LED and photodiode such that they face each other with a small gap between.
    The tab of tape will pass between the LED and photodiode such that it obstructs the light periodically.
    Reverse bias the photodiode with 5V
*/

//LIBRARIES
#include "Arduino.h"
#include <string.h>
#include <SPI.h>
#include <TFT_ST7735.h> //LCD screen
#include <TimerThree.h>        //set PWM freq  http://forum.arduino.cc/index.php?topic=117425.0


// #include <math.h>

#if defined(__SAM3X8E__)
#undef __FlashStringHelper::F(string_literal)
#define F(string_literal) string_literal
#endif


// User Defined Function Declarations
void testArc(uint16_t color);
void lcd_write(String str, uint16_t row);
void esc_val(int throttle);
void int_fired1();
void int_fired2();
void int_fired3();

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
#define __ESC_SIG 2    //ESC signal PWM pin
#define __ESC_FREQ 55  //ESC signal PWM frequency (in Hz)
#define __ESC_PER 18   //ESC signal PWM period (in ms)
#define __INT_RPM1 18   //Interrupt pin for RPM sensor 1
#define __INT_RPM2 19   //Interrupt pin for RPM sensor 2
#define __INT_TACH 20   //Interrupt pin for photodiode of tachometer

#define __CURRENT_LIM 10

#define __AVG_ITERATIONS 50

// Global Variables
int adc_val = 0;
float curr_val = 0.0;
float curr_avg = 0.0;   //Moving average
int pot_val = 0;   //Value maps 0-3.3V to 0-1023 value
int pot_max = 0;
int loop_counter = 0;
int loop_counter2 = 0;

int interrupt_count1 = 0;
int interrupt_count2 = 0;
int interrupt_count3 = 0; //photodiode interrupt counter
unsigned long time_prev = 0;
unsigned long time_curr = 0;
unsigned long time_diff = 0;
float rpm1 = 0.;
float rpm2 = 0.;
float tach = 0.;

bool flag_start = false;

String str;

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

  Serial.println(F("Begin measurement!"));

  lcd_write("Meas #:",1);
  lcd_write("Current:",3);
  lcd_write("ESC val:",5);


  //initialize ESC PWM. 20ms period: 50hz. Logic min: 0.5ms; 1/40 duty. Logic max: 2.5ms; 1/8 duty.
  //**CURRENTLY USED** initialize ESC PWM according to RC controller standard. 18ms period: 55hz. Logic min: 0.5ms; 1/36 duty. Logic max: 2.5ms; 5/36 duty.

  // https://forum.arduino.cc/index.php?topic=72092.0
  // https://forum.arduino.cc/index.php?topic=117425.0
  //Don't change frequency of timer0, associated w/ pins 13,4
  //Pins 2,3,5 associated with timer3
  // InitTimersSafe(); //initialize all timers except for 0, to save time keeping functions
  // bool success = SetPinFrequencySafe(__ESC_SIG, __ESC_FREQ);   //sets the frequency for the specified pin
  // if(success) {
  //   Serial.println(String("PWM freq for pin " + String(__ESC_SIG) + " successfully set to " + String(__ESC_FREQ) + "."));
  // }
  // analogWrite(__ESC_SIG, 255/40); //Initialize ESC PWM to logic min

  // http://playground.arduino.cc/Code/Timer1
  pinMode(__ESC_SIG,OUTPUT);
  Timer3.initialize(__ESC_PER*1000); //set Timer3 period to 18ms. AH_NOTE: this reportedly breaks analogWrite() for pins 9 and 10.
  Timer3.pwm(__ESC_SIG,114);  //duty cycle specified from 0-1023

  //initialize RPM Sensor interrupt pin
  noInterrupts();
  pinMode(__INT_RPM1, INPUT);
  attachInterrupt(digitalPinToInterrupt(__INT_RPM1), int_fired1, RISING);
  pinMode(__INT_RPM2, INPUT);
  attachInterrupt(digitalPinToInterrupt(__INT_RPM2), int_fired2, RISING);
  pinMode(__INT_TACH, INPUT);
  attachInterrupt(digitalPinToInterrupt(__INT_TACH), int_fired3, RISING);

  pot_max = 50;
  lcd_write(String(pot_max),6);
  esc_val(pot_max); // set PPM/PWM signal
}



// ARDUINO MAIN LOOP FUNCTION
void loop(void) {

  //READ JOYSTICK POTENTIOMETER
  adc_val = analogRead(A1); //0-1023 maps to 0-5V. Therefore, 0-675 maps to 0-3.3V
  pot_val = 100-(signed int)((adc_val*100.0)/675.0);
  if ((flag_start == false) && (pot_val > 60)){
    flag_start = true;
    interrupts();
    time_prev = micros();
  }

  //READ CURRENT SENSOR
  adc_val = analogRead(A2);
  // curr_val = (adc_val-100)/8.0 //FOR UNIDIRECTIONAL SENSOR.
  curr_val = (adc_val-510)/4.0; //adjusted 510 to 511 for PCB offset
  curr_avg = (curr_avg + curr_val*3)/4; //Filter using moving average

  //TERMINATING CONDITIONS
  if ((curr_avg > __CURRENT_LIM) || (pot_max > 80)){
      Serial.println("Halting test.");
      pot_max = 50;
      esc_val(pot_max);
      lcd_write(String(pot_max),6);
      loop_counter = 0;
      loop_counter2 = 0;
      flag_start = false;
  }

  //EXECUTE LOGGING
  if (flag_start){
    loop_counter++;
    if (loop_counter >=100){
      //READ RPM SENSOR
      time_curr = micros();
      time_diff = time_curr-time_prev;

      noInterrupts();
      rpm1 += ((float)interrupt_count1 * 1000000.0 / (float)time_diff) * 60.0 / 7.0; //RPM = (interrupts per sec) *60 / (# of poles in motor / 2). Each Torqueboards motor has ...14 poles? AH_NOTE: verify # of poles
      rpm2 += ((float)interrupt_count2 * 1000000.0 / (float)time_diff) * 60.0 / 7.0; //RPM = (interrupts per sec) *60 / (# of poles in motor / 2). Each Torqueboards motor has ...14 poles? AH_NOTE: verify # of poles
      tach += ((float)interrupt_count3 * 1000000.0 / (float)time_diff) * 60.0; //RPM = (# interrupts per sec) *(60 sec per min) / (# of poles in motor / 2). Each Torqueboards motor has ...14 poles? AH_NOTE: verify # of poles
      interrupt_count1 = interrupt_count2 = interrupt_count3 = 0;
      interrupts();

      time_prev = micros();  // store current time for next iteration

      loop_counter = 0;
      loop_counter2++;

      //DISPLAY VALUES ON LCD SCREEN
      lcd_write(String(loop_counter2),2);
      lcd_write(String(curr_avg), 4);
    }

    if (loop_counter2 >= __AVG_ITERATIONS){
      rpm1 /= __AVG_ITERATIONS;
      rpm2 /= __AVG_ITERATIONS;
      tach /= __AVG_ITERATIONS;
      Serial.println(String( String(pot_max)+ " " + String(rpm1) + " " + String(rpm2) + " " + String(tach) ));

      rpm1 = rpm2 = tach = 0;
      loop_counter2 = 0;

      pot_max +=5;
      if (pot_max <=80){
        lcd_write(String(pot_max),6);
        esc_val(pot_max);
        delay(2000);
        interrupt_count1 = interrupt_count2 = interrupt_count3 = 0;

        time_prev = micros();  // store current time for next iteration
      }
    }

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

void esc_val(int throttle){
    //Given value 0-100, sets duty to value scaled between logic min and max.
    //For RC controller, 55hz, 18ms period. Logic min: 1ms; 1/18 duty. Logic max: 2ms; 2/18 duty.
    // analogWrite(__ESC_SIG, (int)(255*( ((float)throttle/100.0)*0.0556 + (0.0556) )) );
    // analogWrite(__ESC_SIG,(int)(127) );
    Timer3.setPwmDuty(__ESC_SIG, 1023*( ((float)throttle/100.0)*0.0556 + (0.0556) ));

}


void int_fired1()
{
    interrupt_count1++;
}
void int_fired2()
{
    interrupt_count2++;
}
void int_fired3()
{
    interrupt_count3++;
}
