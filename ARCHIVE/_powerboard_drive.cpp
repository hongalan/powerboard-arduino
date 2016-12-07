/*
    ALAN HONG
    POWERBOARD PCB CONTROL CODE
    This is the main project file.
*/


//LIBRARIES
#include "Arduino.h"
#include <string.h>
#include <SPI.h>
#include <TFT_ST7735.h> //LCD screen
#include <TimerThree.h>        //set PWM freq  http://forum.arduino.cc/index.php?topic=117425.0
// #include <Adafruit_Sensor.h>
// #include <Adafruit_BNO055.h>
// #include <utility/imumaths.h>
// ROS interface libraries
#include <ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>

#include <avr/interrupt.h>
#include <avr/io.h>


// #include <math.h>

#if defined(__SAM3X8E__)
#undef __FlashStringHelper::F(string_literal)
#define F(string_literal) string_literal
#endif


// User Defined Function Declarations
void testArc(uint16_t color);
void lcd_write(String str, uint16_t row);
void esc_a_val(float throttle_val);
void esc_b_val(float throttle_val);
float rpm_to_mps(float rpm_val);
float pot_to_mps(float pot_val);
void int_fired1();
void int_fired2();
void pcb_poweron();
void pcb_shutdown();
void estop_close();
void estop_open();


// Color definitions
#define	BLUE    0x001F
#define	RED     0xF800
#define	GREEN   0x07E0
#define CYAN    0x07FF
#define MAGENTA 0xF81F
#define YELLOW  0xFFE0

// Pin Mapping
#define __CS 	49       //SPI Chip Select pin
#define __DC 	48       //SPI Data pin
#define __RST 	255    //SPI Reset (not used)
#define __ESC_A_SIG 2    //ESC A signal PWM pin
#define __ESC_B_SIG 3    //ESC B signal PWM pin
#define __ESC_FREQ 55  //ESC signal PWM frequency (in Hz)
#define __ESC_PER 18   //ESC signal PWM period (in ms)
#define __INT_RPM1 18   //Interrupt pin for RPM sensor 1
#define __INT_RPM2 19   //Interrupt pin for RPM sensor 2
#define __MPS_REF_INCREMENT 0.05
#define __ISENS1_ADC A1  //current sensor 1 ADC pin
#define __ISENS2_ADC A2  //current sensor 2 ADC pin
#define __PWR_ADC A3  //battery voltage ADC pin
#define __POT_ADC A4  //joystick potentiometer ADC pin


#define __V_REG 46    //5V regulator enable pin (write hi to disable)
#define __SPARE 47    // Spare optocoupler "G"
#define __MOS_G 44    //Mosfet gate pin (must be written hi to close circuit)
// #define __BDC_G 45    //Brushed motor mosfet gate pin (must be written hi to close circuit)

#define __MPS_MAX 2    //Maximum speed in meters per second
#define __THROTTLE_MAX 1000   //Sets scale for signal representation

//PID gains
#define __KP 10
#define __KD 15
#define __KI 0
#define __MPS_I_ERR_CAP 1

#define __TIMER_P_MS 10 //Specifies desired period for timer interrupt

#define __CURRENT_LIM 10

// Global Variables
int adc_read = 0;
float curr_read = 0.0;
float curr_avg = 0.0;   //Moving average
float pot_read = 0;
float pot_avg = 0;   //Moving average
unsigned int batt_voltage_count = 0;

unsigned int timer_int_count = 0; //Counter for timer interrupt

int interrupt_count1 = 0;
int interrupt_count2 = 0;
unsigned long time_prev = 0;
unsigned long time_curr = 0;
unsigned long time_diff = 0;
float rpm1 = 0;
float rpm2 = 0;
float mps1 = 0;
float mps2 = 0;

float mps_ref = 0;
float mps1_e = 0;
float mps1_e_d = 0;
float mps1_e_i = 0;
float mps2_e = 0;
float mps2_e_d = 0;
float mps2_e_i = 0;
float throttle1 = __THROTTLE_MAX/2;   //Value written to ESC
float throttle2 = __THROTTLE_MAX/2;
float throttle1_safe = __THROTTLE_MAX/2;;   //Last acceptable pot_read within current limit
float throttle2_safe = __THROTTLE_MAX/2;;

// Adafruit_BNO055 bno = Adafruit_BNO055(55);
// ros::NodeHandle nh;
// std_msgs::String str_msg;
// ros::Publisher test_chatter("test_chatter_topic", &str_msg);
// char hello[13] = "hello world!";
// sensor_msgs::Imu imu_msg;
// ros::Publisher imu_pub("imu_topic", &imu_msg);
// geometry_msgs::Vector3 wheel_msg;
// ros::Publisher wheel_pub("wheel_topic", &wheel_msg);


TFT_ST7735 tft = TFT_ST7735(__CS, __DC, __RST);


//TIMER INTERRUPT
//Timer2 Overflow Interrupt Vector, called every 1ms
ISR(TIMER2_OVF_vect) {
  timer_int_count++;               //Increments the interrupt counter
  if(timer_int_count > __TIMER_P_MS){

    //Calculate rpm and mps
    time_curr = micros();
    time_diff = time_curr-time_prev;
    // noInterrupts();
    rpm1 = (4*rpm1 + ( ((float)interrupt_count1 * 1000000.0 / (float)time_diff) * 60.0 / 7.0 ))/5; //RPM = (interrupts per sec) *60 / (# of poles in motor). Ordinarily you'd multiply this by 2, but since we're counting interrupts by CHANGE and not RISING or FALLING, we've essentially already done so.  Each Torqueboards motor has ...14 poles? AH_NOTE: verify # of poles
    rpm2 = (4*rpm2 + ( ((float)interrupt_count2 * 1000000.0 / (float)time_diff) * 60.0 / 7.0 ))/5; //RPM = (interrupts per sec) *60 / (# of poles in motor / 2). Each Torqueboards motor has ...14 poles? AH_NOTE: verify # of poles
    interrupt_count1 = 0;
    interrupt_count2 = 0;
    // interrupts();
    time_prev = micros();  // store current time for next iteration

    mps1 = rpm_to_mps(rpm1);
    mps2 = rpm_to_mps(rpm2);


    //PID error calculation
    mps1_e_d = (mps_ref - mps1) - mps1_e; //diffence in error between iterations
    mps1_e = mps_ref - mps1;  //current error
    mps1_e_i = (mps1_e * 0.75) + mps1_e; //integrated error... with a term thrown in to make sure it doesn't persist unduly and cause the control to drift
    if (fabs(mps1_e_i) > __MPS_I_ERR_CAP){
      if (signbit(mps1_e_i)){
        mps1_e_i = - __MPS_I_ERR_CAP;
      } else {
        mps1_e_i = __MPS_I_ERR_CAP;
      }
    }
    mps2_e_d = (mps_ref - mps2) - mps2_e; //diffence in error between iterations
    mps2_e = mps_ref - mps2;  //current error
    mps2_e_i = (mps2_e * 0.75) + mps2_e; //integrated error... with a term thrown in to make sure it doesn't persist unduly and cause the control to drift
    if (fabs(mps2_e_i) > __MPS_I_ERR_CAP){
      if (signbit(mps2_e_i)){
        mps2_e_i = - __MPS_I_ERR_CAP;
      } else {
        mps2_e_i = __MPS_I_ERR_CAP;
      }
    }

    //Error-adjusted throttle
    throttle1 += __KP * mps1_e + __KI * mps1_e_i + __KD * mps1_e_d;
    throttle2 += __KP * mps2_e + __KI * mps2_e_i + __KD * mps2_e_d;
    //Safety conditions:
    if (curr_avg < -0.5){ //reverse current
        throttle1 = (throttle1+ (__THROTTLE_MAX/2)*4 )/5; //drag written value down to neutral
        throttle1_safe = __THROTTLE_MAX/2;
        throttle2 = (throttle2+ (__THROTTLE_MAX/2)*4 )/5; //drag written value down to neutral
        throttle2_safe = __THROTTLE_MAX/2;
    } else if (curr_avg <= __CURRENT_LIM){
        throttle1_safe = (throttle1 + 4*throttle1_safe)/5;
        throttle2_safe = (throttle2 + 4*throttle2_safe)/5;
    } else if ((throttle1_safe > __THROTTLE_MAX/2) && (throttle2_safe > __THROTTLE_MAX/2)){ //overcurrent and not moving backward
        throttle1 = throttle1_safe - __THROTTLE_MAX/100;
        throttle2 = throttle2_safe - __THROTTLE_MAX/100;
    }
    if (fabs(pot_avg-__THROTTLE_MAX/2) <__THROTTLE_MAX*0.01){ //brake
      throttle1 = __THROTTLE_MAX/2 + (throttle1-(__THROTTLE_MAX/2))*0.5; //drag written value down to neutral
      throttle2 = __THROTTLE_MAX/2 + (throttle2-(__THROTTLE_MAX/2))*0.5;
    }

    esc_a_val(throttle1); // set PPM/PWM signal for ESC signal A
    esc_b_val(throttle2); // set PPM/PWM signal for ESC signal B

    timer_int_count = 0;           //Resets the interrupt counter
  }
  TCNT2 = 130;           //Reset Timer to 130 out of 255
  TIFR2 = 0x00;          //Timer2 INT Flag Reg: Clear Timer Overflow Flag
};

// ARDUINO MAIN SETUP FUNCTION
void setup() {
  Serial.begin(9600);
  tft.begin();

  testArc(GREEN);
  tft.clearScreen();
  tft.setTextColor(WHITE);
  tft.setTextSize(2);

  // Serial.println(F("Let's take some data"));
  lcd_write("Amps",1);
  lcd_write("MPS_ref",3);
  lcd_write("MPS_avg",5);

  //initialize ESC PWM. 20ms period: 50hz. Logic min: 0.5ms; 1/40 duty. Logic max: 2.5ms; 1/8 duty.
  //**CURRENTLY USED** initialize ESC PWM according to RC controller standard. 18ms period: 55hz. Logic min: 0.5ms; 1/36 duty. Logic max: 2.5ms; 5/36 duty.
  pinMode(__ESC_A_SIG,OUTPUT);
  pinMode(__ESC_B_SIG,OUTPUT);
  Timer3.initialize(__ESC_PER*1000); //set Timer3 period to 18ms. AH_NOTE: this reportedly breaks analogWrite() for pins 9 and 10.
  Timer3.pwm(__ESC_A_SIG,114);  //duty cycle specified from 0-1023
  Timer3.pwm(__ESC_B_SIG,114);  //duty cycle specified from 0-1023

  pinMode(__V_REG,OUTPUT);
  pcb_poweron(); //ensures that enable pin of pcb 5v regulator remains hi with its integrated pull-up resistor
  // lcd_write ("ON",2);
  pinMode(__MOS_G,OUTPUT);
  estop_close();
  // lcd_write ("ON",4);

  // Initialize node handle
  // nh.initNode();
  // nh.advertise(test_chatter);
  // nh.advertise(imu_pub);
  // nh.advertise(wheel_pub);

  // Initialise the IMU
  // if(!bno.begin())
  // {
  //   /* There was a problem detecting the BNO055 ... check your connections */
  //   tft.clearScreen();
  //   lcd_write(String("IMU err"), 1);
  //   while(1);
  // }
  // bno.setExtCrystalUse(true);


  noInterrupts();

  //Setup Timer2 to fire every 1ms
    TCCR2B = 0x00;        //Disable Timer2 while we set it up
    TCNT2  = 130;         //Reset Timer Count to 130 out of 255
    TIFR2  = 0x00;        //Timer2 INT Flag Reg: Clear Timer Overflow Flag
    TIMSK2 = 0x01;        //Timer2 INT Reg: Timer2 Overflow Interrupt Enable
    TCCR2A = 0x00;        //Timer2 Control Reg A: Normal port operation, Wave Gen Mode normal
    TCCR2B = 0x05;        //Timer2 Control Reg B: Timer Prescaler set to 128

  //initialize RPM Sensor interrupt pin
  interrupt_count1 = 0;
  interrupt_count2 = 0;
  pinMode(__INT_RPM1, INPUT);
  attachInterrupt(digitalPinToInterrupt(__INT_RPM1), int_fired1, RISING);
  pinMode(__INT_RPM2, INPUT);
  attachInterrupt(digitalPinToInterrupt(__INT_RPM2), int_fired2, RISING);
  time_prev = micros();

  interrupts();
}



// ARDUINO MAIN LOOP FUNCTION
void loop(void) {

  //READ JOYSTICK POTENTIOMETER
  adc_read = analogRead(__POT_ADC); //0-1023 maps to 0-5V. Therefore, 0-675 maps to 0-3.3V
  pot_read = __THROTTLE_MAX-(((float)(adc_read)/675.0) * __THROTTLE_MAX);
  if (pot_read < __THROTTLE_MAX/2){
    pot_read = __THROTTLE_MAX/2;
    //pot_read = ((1.5*__THROTTLE_MAX)-pot_read)/2; //allows for twice the joystick resolution in one direction. i.e.: neutral to __THROTTLE_MAX in one direction, neutral to 0.75*THROTTLE_MAX in the other
  }
  if (fabs(pot_read-__THROTTLE_MAX/2) < __THROTTLE_MAX * 0.01){
    pot_read = __THROTTLE_MAX/2; //if potentiometer value is within 1% from the neutral position, assume neutral.
  }
  pot_avg = (pot_avg + 4*pot_read)/5; //Filter using moving average
  mps_ref = min(pot_to_mps(pot_avg),((mps1 + mps2)*0.5) + __MPS_REF_INCREMENT );


  //Read current sensor
  adc_read = analogRead(__ISENS1_ADC);
  curr_read = (adc_read-100)/8.0; //FOR UNIDIRECTIONAL SENSOR.
  // curr_read = (adc_read-510)/4.0; //FOR BI-DIRECTIONAL SENSOR; adjusted 510 for observed zero-current offset
  curr_avg = (curr_avg + 4*curr_read)/5; //Filter using moving average

  //Read battery voltage
  adc_read = analogRead(__PWR_ADC);
  // lcd_write(String(((((float)adc_read) /1023.0)*5.0)), 2);
  //battery voltage is split such that voltage read is 0.190283401 * (battery voltage)
  if ( ((((double)adc_read) /1023.0)*5.0) < (0.190283401 * 21.0) ) { //if voltage dips below 3.6v per cell average
    batt_voltage_count++;
    if (batt_voltage_count>200){
      esc_a_val(__THROTTLE_MAX/2 );
      esc_b_val(__THROTTLE_MAX/2 );
      estop_open();
      pcb_shutdown();
      tft.clearScreen();
      lcd_write("Batt lo",1);
      lcd_write(String(((((float)adc_read) /1023.0)*5.0)), 2);
      noInterrupts();
      while (((((float)adc_read) /1023.0)*5.0) < (0.190283401 * 21.6)){
        adc_read = analogRead(__PWR_ADC);
        lcd_write(String(((((float)adc_read) /1023.0)*5.0)), 2);
      }
    }
  } else{
    batt_voltage_count = 0;
  }

  /*SENSOR DATA TO ROS*/
  /* Get a new sensor event */
  // sensors_event_t event;
  // bno.getEvent(&event);
  // imu::Quaternion quat = bno.getQuat();
  // imu_msg.orientation.x = quat.x();
  // imu_msg.orientation.y = quat.y();
  // imu_msg.orientation.z = quat.z();
  // imu_msg.orientation.w = quat.w();
  // str_msg.data = hello;
  // // rpm1 and mps1 assumed to measure left motor, rpm2 and mps2 assumed to measure right motor
  // wheel_msg.x = mps1;
  // wheel_msg.y = mps2;
  // wheel_msg.z = time_curr*0.000001; //counting on double precision, 64 bits, to represent time in seconds
  // imu_pub.publish ( &imu_msg );
  // test_chatter.publish( &str_msg );
  // wheel_pub.publish( &wheel_msg );
  // nh.spinOnce();
  //
  // lcd_write(String(curr_avg), 2);

  lcd_write(String(throttle1),1);
  lcd_write(String(throttle2),2);

  lcd_write(String(mps_ref),4);
  lcd_write(String(mps1),5);
  lcd_write(String(mps2),6);

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

void esc_a_val(float throttle_val){
    //Given value 0-100, sets duty to value scaled between logic min and max.
    //For RC controller, 55hz, 18ms period. Logic min: 1ms; 1/18 duty. Logic max: 2ms; 2/18 duty.
    // analogWrite(__ESC_SIG, (int)(255*( (throttle_val/__THROTTLE_MAX)*0.0556 + (0.0556) )) );
    Timer3.setPwmDuty(__ESC_A_SIG, (float)(1023.0*( (throttle_val/__THROTTLE_MAX)*0.0556 + (0.0556) )) );
}

void esc_b_val(float throttle_val){
    //Given value 0-100, sets duty to value scaled between logic min and max.
    //For RC controller, 55hz, 18ms period. Logic min: 1ms; 1/18 duty. Logic max: 2ms; 2/18 duty.
    // analogWrite(__ESC_SIG, (int)(255*( (throttle_val/__THROTTLE_MAX)*0.0556 + (0.0556) )) );
    Timer3.setPwmDuty(__ESC_B_SIG, (float)(1023.0*( (throttle_val/__THROTTLE_MAX)*0.0556 + (0.0556) )) );
}

float rpm_to_mps(float rpm_val){
    // 13 to 36 gear ratio -> 1 motor_rev = 13/36 wheel_rev
    // Wheel diameter is 3.269" (0.0830326m)-> 1 wheel_rev = 0.0830326*pi meters
    // 1 min = 60 sec
    // 1 (motor_rev/min) * 13/36 (wheel_rev/motor_rev) * 0.0830326*pi (meters/wheel_rev) * 1/60 (min/sec) -> 0.001569912 m/s
    return (rpm_val * 0.001569912);
}

float pot_to_mps(float pot_val){
  //sets desired longboard speed according to pot_val
  //takes pot_val from 0-__THROTTLE_MAX, maps 0-__THROTTLE_MAX/2 to 0, maps __THROTTLE_MAX/2 - __THROTTLE_MAX to 0-(__MPS_MAX)
  if (pot_val <= __THROTTLE_MAX/2){
    return 0;
  } else {
    return (pot_val-__THROTTLE_MAX/2)/(__THROTTLE_MAX/2) * __MPS_MAX;
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

void pcb_shutdown(){
  digitalWrite(__V_REG, HIGH); //by writing hi, optocoupler closes connection from GND to 5v regulator's enable pin, pulling it to lo and disabling regulator, disabling functionality since the MOSFET gates are driven by the regulator
}
void pcb_poweron(){
  digitalWrite(__V_REG,LOW);
}
void estop_close(){
  digitalWrite(__MOS_G,HIGH);
}
void estop_open(){
  digitalWrite(__MOS_G,LOW);
}
