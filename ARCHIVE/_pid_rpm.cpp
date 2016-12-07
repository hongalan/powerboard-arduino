/*
    ALAN HONG
    POWERBOARD PID CONTROL CODE
    This sets a reference speed and writes ESC signal according to simple feedback loop.
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
void esc_val(float throttle_val);
float rpm_to_fps(float rpm_val);
float pot_to_fps(float pot_val);
void int_fired1();
void int_fired2();

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

#define __FPS_MAX 10    //Maximum speed in feet per second
#define __THROTTLE_MAX 1000   //Sets scale for signal representation

//PID gains
#define __KP 0.1
#define __KD 0.08
#define __KI 0
#define __FPS_I_ERR_CAP 1

#define __CURRENT_LIM 10

// Global Variables
int adc_read = 0;
float curr_read = 0.0;
float curr_avg = 0.0;   //Moving average
float pot_read = 0;
float pot_avg = 0;   //Moving average
float throttle_safe = 0;   //Last acceptable pot_read within current limit

int loop_counter = 0;

int interrupt_count1 = 0;
int interrupt_count2 = 0;
unsigned long time_prev = 0;
unsigned long time_curr = 0;
unsigned long time_diff = 0;
float rpm1 = 0;
float rpm2 = 0;
float fps1 = 0;
float fps2 = 0;

float fps_ref = 0;
float fps1_e = 0;
float fps1_e_d = 0;
float fps1_e_i = 0;
float throttle = __THROTTLE_MAX/2;   //Value written to ESC


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

  lcd_write("fps_ref:",1);
  lcd_write("Cur val:",3);
  lcd_write("fps1:",5);

  Serial.println(F("...soon."));


  //initialize ESC PWM. 20ms period: 50hz. Logic min: 0.5ms; 1/40 duty. Logic max: 2.5ms; 1/8 duty.
  //**CURRENTLY USED** initialize ESC PWM according to RC controller standard. 18ms period: 55hz. Logic min: 0.5ms; 1/36 duty. Logic max: 2.5ms; 5/36 duty.
  pinMode(__ESC_SIG,OUTPUT);
  Timer3.initialize(__ESC_PER*1000); //set Timer3 period to 18ms. AH_NOTE: this reportedly breaks analogWrite() for pins 9 and 10.
  Timer3.pwm(__ESC_SIG,114);  //duty cycle specified from 0-1023

  //initialize RPM Sensor interrupt pin
  noInterrupts();
  pinMode(__INT_RPM1, INPUT);
  attachInterrupt(digitalPinToInterrupt(__INT_RPM1), int_fired1, RISING);
  pinMode(__INT_RPM2, INPUT);
  attachInterrupt(digitalPinToInterrupt(__INT_RPM2), int_fired2, RISING);
  interrupts();
  time_prev = micros();
}



// ARDUINO MAIN LOOP FUNCTION
void loop(void) {

  //READ JOYSTICK POTENTIOMETER
  adc_read = analogRead(A1); //0-1023 maps to 0-5V. Therefore, 0-675 maps to 0-3.3V
  pot_read = __THROTTLE_MAX-(((float)(adc_read)/675.0) * __THROTTLE_MAX);
  if (pot_read < __THROTTLE_MAX/2){
    pot_read = ((1.5*__THROTTLE_MAX)-pot_read)/2; //allows for twice the joystick resolution in one direction. i.e.: neutral to __THROTTLE_MAX in one direction, neutral to 0.75*THROTTLE_MAX in the other
  }
  if (fabs(pot_read-__THROTTLE_MAX/2) < __THROTTLE_MAX * 0.01){
    pot_read = __THROTTLE_MAX/2; //if potentiometer value is within 1% from the neutral position, assume neutral.
  }

  pot_avg = (pot_avg + 4*pot_read)/5; //Filter using moving average

  fps_ref = pot_to_fps(pot_avg);


  //READ CURRENT SENSOR
  adc_read = analogRead(A2);
  // curr_read = (adc_read-100)/8.0 //FOR UNIDIRECTIONAL SENSOR.
  curr_read = (adc_read-510)/4.0; //adjusted 510 for observed zero-current offset
  curr_avg = (curr_avg + 4*curr_read)/5; //Filter using moving average

  //CALCULATE RPM and FPS
  time_curr = micros();
  time_diff = time_curr-time_prev;
  noInterrupts();
  rpm1 = (4*rpm1 + ( ((float)interrupt_count1 * 1000000.0 / (float)time_diff) * 60.0 / 7.0 ))/5; //RPM = (interrupts per sec) *60 / (# of poles in motor). Ordinarily you'd multiply this by 2, but since we're counting interrupts by CHANGE and not RISING or FALLING, we've essentially already done so.  Each Torqueboards motor has ...14 poles? AH_NOTE: verify # of poles
  rpm2 = (4*rpm2 + ( ((float)interrupt_count2 * 1000000.0 / (float)time_diff) * 60.0 / 7.0 ))/5; //RPM = (interrupts per sec) *60 / (# of poles in motor / 2). Each Torqueboards motor has ...14 poles? AH_NOTE: verify # of poles
  interrupt_count1 = 0;
  interrupt_count2 = 0;
  interrupts();
  time_prev = micros();  // store current time for next iteration

  fps1 = rpm_to_fps(rpm1);
  fps2 = rpm_to_fps(rpm2);


  //PID error calculation
  fps1_e_d = (fps_ref - fps1) - fps1_e; //diffence in error between iterations
  fps1_e = fps_ref - fps1;  //current error
  fps1_e_i = (fps1_e * 0.75) + fps1_e; //integrated error... with a term thrown in to make sure it doesn't persist unduly and cause the control to drift
  if (fabs(fps1_e_i) > __FPS_I_ERR_CAP){
    if (signbit(fps1_e_i)){
      fps1_e_i = - __FPS_I_ERR_CAP;
    } else {
      fps1_e_i = __FPS_I_ERR_CAP;
    }
  }

  //Error-adjusted throttle
  throttle += __KP * fps1_e + __KI * fps1_e_i + __KD * fps1_e_d;

  //Safety conditions:
  if (curr_avg < -0.5){ //reverse current
      throttle = (throttle+ (__THROTTLE_MAX/2)*4 )/5; //drag written value down to neutral
      throttle_safe = __THROTTLE_MAX/2;
  } else if (curr_avg <= __CURRENT_LIM){
      throttle_safe = (throttle + 4*throttle_safe)/5;
  } else if (throttle_safe > __THROTTLE_MAX/2){ //overcurrent
      throttle = throttle_safe - __THROTTLE_MAX/100;
  }

  if (fabs(pot_avg-__THROTTLE_MAX/2) <__THROTTLE_MAX*0.01){ //brake
    throttle = __THROTTLE_MAX/2 + (throttle-(__THROTTLE_MAX/2))*0.5; //drag written value down to neutral
  }

  esc_val(throttle); // set PPM/PWM signal


  //DISPLAY VALUES
  // loop_counter++;
  // if (loop_counter >=50){

    // lcd_write(String(fps_ref),2);
    // lcd_write(String(curr_avg), 4);
    // lcd_write(String(fps1),6);

    // Serial.println(String("pot_read: " + String(pot_read) + " | fps_ref: " + String(fps_ref) ));
    // Serial.println(String("p_e: " + String(fps1_e) + " d_e: " + String(fps1_e_d) + " i_e: " + String (fps1_e_i)));
    // Serial.println(String("throttle: " + String (throttle) + " | throttle_safe: " + String(throttle_safe)) );

    // Serial.println(String(String(fps_ref) + " " + String(fps1) + " " + String(fps2) ));
    // Serial.println(String( "ERR: "+ String(__KP * fps1_e + __KI * fps1_e_i + __KD * fps1_e_d) + " | ESC : " + String( (1023.0*( (throttle/__THROTTLE_MAX)*0.0556 + (0.0556))) ))) ;
  //
  //   loop_counter = 0;
  // }

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

void esc_val(float throttle_val){
    //Given value 0-100, sets duty to value scaled between logic min and max.
    //For RC controller, 55hz, 18ms period. Logic min: 1ms; 1/18 duty. Logic max: 2ms; 2/18 duty.
    // analogWrite(__ESC_SIG, (int)(255*( (throttle_val/__THROTTLE_MAX)*0.0556 + (0.0556) )) );
    Timer3.setPwmDuty(__ESC_SIG, (float)(1023.0*( (throttle_val/__THROTTLE_MAX)*0.0556 + (0.0556) )) );

}

float rpm_to_fps(float rpm_val){
    // 13 to 36 gear ratio -> 1 motor_rev = 13/36 wheel_rev
    // Wheel diameter is 3.269" -> 1 wheel_rev = 3.269*pi/12 ft
    // 1 min = 60 sec
    // 1 (motor_rev/min) * 13/36 (wheel_rev/motor_rev) * 3.269*pi/12 (ft/wheel_rev) * 1/60 (min/sec) -> 0.00515063 ft/s
    return (rpm_val * 0.00515063);
}

float pot_to_fps(float pot_val){
  //sets desired longboard speed according to pot_val
  //takes pot_val from 0-__THROTTLE_MAX, maps 0-__THROTTLE_MAX/2 to 0, maps __THROTTLE_MAX/2 - __THROTTLE_MAX to 0-(__FPS_MAX)
  if (pot_val <= __THROTTLE_MAX/2){
    return 0;
  } else {
    return (pot_val-__THROTTLE_MAX/2)/(__THROTTLE_MAX/2) * __FPS_MAX;
  }

}

void int_fired1()
{
    interrupt_count1++;
}
void int_fired2()
{
    interrupt_count2++;
}
