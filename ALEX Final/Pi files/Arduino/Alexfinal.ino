//#include <ros.h>
#include "serialize.h"
#include "packet.h"
#include "constants.h"
#include <stdarg.h>
#include <math.h>
#include <avr/sleep.h>
#include <Wire.h>
#include "SparkFunISL29125.h"
#define trigPin 8
#define echoPin 9

#define S0 0
#define S1 1
#define S2 7
#define S3 12
#define sensorOut 13

int frequency = 0;
int red = 0;
int green = 0;
int blue = 0;

// Declare sensor object
SFE_ISL29125 RGB_sensor;
typedef enum
{
  STOP = 0,
  FORWARD = 1,
  BACKWARD = 2,
  LEFT = 3,
  RIGHT = 4
} TDirection;

volatile TDirection dir = STOP;

/*
   Alex's configuration constants
*/

#define PRR_TWI_MASK 0b10000000
#define PRR_SPI_MASK 0b00000100
#define ADCSRA_ADC_MASK 0b10000000
#define PRR_ADC_MASK 0b00000001
#define PRR_TIMER2_MASK 0b01000000
#define PRR_TIMER0_MASK 0b00100000
#define PRR_TIMER1_MASK 0b00001000
#define SMCR_SLEEP_ENABLE_MASK 0b00000001
#define SMCR_IDLE_MODE_MASK 0b11110001

//for calculating circumference
//#define PI 3.141592654
#define ALEX_LENGTH         16
#define ALEX_BREADTH        6

float alexDiagonal = 0.0;
float alexCirc = 0.0;

// Number of ticks per revolution from the
// wheel encoder.
#define COUNTS_PER_REV      180

// Wheel circumference in cm.
// We will use this to calculate forward/backward distance traveled
// by taking revs * WHEEL_CIRC
#define WHEEL_CIRC          20.42

// Motor control pins. You need to adjust these till
// Alex moves in the correct direction
#define LF                  6   // Left forward pin
#define LR                  5   // Left reverse pin
#define RF                  10  // Right forward pin
#define RR                  11  // Right reverse pin

// Ultrasonic Sensor Pins:
// Declaration of the DIGITAL sensor input pins


//IR Sensor Pins:
/*
  #define LeftIR A1
  #define RightIR A2
  #define RearIR A3
*/
#define LF_IR A0
#define LR_IR A2
#define RF_IR A1
#define RR_IR A3
#define RearIR 4


//Colour Sensor Pins:
/*
  #define Colour_s0 4       //Module pins wiring
  #define Colour_s1 7
  #define Colour_s2 12
  #define Colour_s3 13
  #define Colour_out A0
*/
/*
  #define Colour_s0 A0       //Module pins wiring
  #define Colour_s1 A1
  #define Colour_s2 A2
  #define Colour_s3 A3
  #define Colour_out A4
*/

//Ultrasonic
volatile float duration = 0;
volatile unsigned long ultrasonic_distance = 0;
volatile unsigned long prev_ultrasonic_dist = 0;


//IR
volatile int LF_IR_Val = 0;
volatile int LR_IR_Val = 0;
volatile int RF_IR_Val = 0;
volatile int RR_IR_Val = 0;
volatile int Rear_IR_Val = 0;
//volatile unsigned long LF_Val = 0;
//volatile unsigned long LR_Val = 0;
//volatile unsigned long RF_Val = 0;
//volatile unsigned long RR_Val = 0;
//volatile unsigned long Rear_Val = 0;
volatile unsigned long ir_val = 0;
//Wheels
//Left and Right wheel distance
volatile unsigned long leftDist = 0;
volatile unsigned long rightDist = 0;
volatile unsigned long prev_leftDist = 0;
volatile unsigned long prev_rightDist = 0;

//Main Loop:
unsigned long prev_time = 0;
unsigned long new_time;

// Store the ticks from Alex's left and
// right encoders for moving forwards and backwards
volatile unsigned long leftForwardTicks;
volatile unsigned long rightForwardTicks;
volatile unsigned long leftReverseTicks;
volatile unsigned long rightReverseTicks;

// Store the ticks from Alex's left and
// right encoders for turning
volatile unsigned long leftForwardTicksTurns;
volatile unsigned long rightForwardTicksTurns;
volatile unsigned long leftReverseTicksTurns;
volatile unsigned long rightReverseTicksTurns;

// Store the revolutions on Alex's left
// and right wheels
volatile unsigned long leftRevs;
volatile unsigned long rightRevs;

// Forward and backward distance traveled
volatile unsigned long forwardDist;
volatile unsigned long reverseDist;

//Variables to keep track of whether Alex has move a commmanded distance
unsigned long deltaDist;
unsigned long newDist;

unsigned long deltaTicks;
unsigned long targetTicks;

//Colour Sensor Colour array
int ColourData[] = {0, 0, 0};        //{RED, BLUE, GREEN}
//char Colour;
int Colour = 0;

int red_total = 0;
int green_total = 0;
int blue_total = 0;
int x;

int looper = 0;

volatile unsigned long col_reading = 3;


void WDT_off(void)
{
  /* Global interrupt should be turned OFF here if not
    already done so */
  cli();
  /* Clear WDRF in MCUSR */
  MCUSR &= ~(1 << WDRF);
  /* Write logical one to WDCE and WDE */
  /* Keep old prescaler setting to prevent unintentional
    time-out */
  WDTCSR |= (1 << WDCE) | (1 << WDE);
  /* Turn off WDT */
  WDTCSR = 0x00;
  /* Global interrupt should be turned ON here if
    subsequent operations after calling this function DO
    NOT require turning off global interrupt */
  sei();
}

//Function to set up power saving
void setupPowerSaving()
{
  // Watchdog Timer OFF
  WDT_off();
  //Shut down TWI
  PRR |= PRR_TWI_MASK;
  //Shut down SPI
  PRR |= PRR_SPI_MASK;
  //Disable ADC,
  ADCSRA &= ~ADCSRA_ADC_MASK;
  //Shut down ADC
  PRR |= PRR_ADC_MASK;
  // Set the SMCR to choose the IDLE sleep mode
  // Do not set the Sleep Enable (SE) bit yet
  SMCR &= SMCR_IDLE_MODE_MASK;
  // Set Port B Pin 5 as output pin, then write a logic LOW
  // to it so that the LED tied to Arduino's Pin 13 is OFF.
  DDRB |= B11011111;
  PORTB &= B11011111;
}

void putArduinoToIdle()
{
  //Shut down TIMER 0, 1, and 2
  PRR |= (PRR_TIMER0_MASK | PRR_TIMER1_MASK | PRR_TIMER2_MASK);
  // Modify SE bit in SMCR to enable (i.e., allow) sleep
  SMCR |= SMCR_SLEEP_ENABLE_MASK;
  // This function puts ATmega328P’s MCU into sleep
  sleep_cpu();
  // Modify SE bit in SMCR to disable (i.e., disallow) sleep
  SMCR &= ~SMCR_SLEEP_ENABLE_MASK;
  // Modify PRR to power up TIMER 0, 1, and 2
  PRR &= ~(PRR_TIMER0_MASK | PRR_TIMER1_MASK | PRR_TIMER2_MASK);
}

TResult readPacket(TPacket *packet)
{
  // Reads in data from the serial port and
  // deserializes it.Returns deserialized
  // data in "packet".

  char buffer[PACKET_SIZE];
  int len;

  len = readSerial(buffer);

  if (len == 0) {
    return PACKET_INCOMPLETE;
  }

  else
    return deserialize(buffer, len, packet);

}

/*
  void sendColour(int colour)
  {
  //sends colour "status" when findColour is called on the pi

  TPacket statusPacket;
  statusPacket.packetType = PACKET_TYPE_RESPONSE;
  statusPacket.command = RESP_COLOUR;
  statusPacket.params[0] = colour;

  sendResponse(&statusPacket);
  }
*/

void sendColour()
{
  TPacket colourPacket;
  colourPacket.packetType = PACKET_TYPE_RESPONSE;
  colourPacket.command = RESP_COLOUR;

  colourPacket.params[0] = col_reading;            //send colour detected back as a string
  sendResponse(&colourPacket);
}

//to be edited:
//sends Colour [1 bit],
//Ultrasonic data in cm [1 bit] - limit interval to 0.5cm
// proximity ir sensors [3 bits], wheel ticks distance for left and right wheel [up to 2 bits]
//add functions for reporting of ir proximity + object detection + ticks (could be for forward/reverse or left/right)
void sendUltrasonicIRinfo() {

  TPacket UltrasonicDistPacket;
  UltrasonicDistPacket.packetType = PACKET_TYPE_RESPONSE;
  UltrasonicDistPacket.command = RESP_ULTRASONIC;

  UltrasonicDistPacket.params[0] = ultrasonic_distance;
  UltrasonicDistPacket.params[1] = ir_val;
  UltrasonicDistPacket.params[2] = col_reading;

  sendResponse(&UltrasonicDistPacket);
}

void sendIR() {

  TPacket IRPacket;
  IRPacket.packetType = PACKET_TYPE_RESPONSE;
  IRPacket.command = RESP_IR;
  IRPacket.params[1] = ir_val;
  /*IRPacket.params[0] = LF_IR_Val;
    IRPacket.params[1] = LR_IR_Val;
    IRPacket.params[2] = RF_IR_Val;
    IRPacket.params[3] = RR_IR_Val;
    IRPacket.params[4] = Rear_IR_Val;*/

  sendResponse(&IRPacket);
}

void sendWheelDist() {

  TPacket WheelDistPacket;
  WheelDistPacket.packetType = PACKET_TYPE_RESPONSE;
  WheelDistPacket.command = RESP_WHEEL;

  WheelDistPacket.params[0] = leftDist;
  WheelDistPacket.params[1] = rightDist;

  sendResponse(&WheelDistPacket);
}

//can remove later if dun want the 'status' functionality
void sendStatus()
{
  // Implement code to send back a packet containing key
  // information like leftTicks, rightTicks, leftRevs, rightRevs
  // forwardDist and reverseDist
  // Use the params array to store this information, and set the
  // packetType and command files accordingly, then use sendResponse
  // to send out the packet. See sendMessage on how to use sendResponse.
  //

  TPacket statusPacket;
  statusPacket.packetType = PACKET_TYPE_RESPONSE;
  statusPacket.command = RESP_STATUS;
  //statusPacket.params[0] = ultrasonic_distance;
  /*statusPacket.params[0] = leftForwardTicks;
    statusPacket.params[1] = rightForwardTicks;
    statusPacket.params[2] = leftReverseTicks;
    statusPacket.params[3] = rightReverseTicks;
    statusPacket.params[4] = leftForwardTicksTurns;
    statusPacket.params[5] = leftReverseTicksTurns;
    statusPacket.params[6] = rightForwardTicksTurns;
    statusPacket.params[7] = rightReverseTicksTurns;
    statusPacket.params[8] = forwardDist;
    statusPacket.params[9] = reverseDist;*/

  sendResponse(&statusPacket);
}

void sendMessage(const char *message)
{
  // Sends text messages back to the Pi. Useful
  // for debugging.

  TPacket messagePacket;
  messagePacket.packetType = PACKET_TYPE_MESSAGE;
  strncpy(messagePacket.data, message, MAX_STR_LEN);
  sendResponse(&messagePacket);
}


void dbprintf(char* format, ...) {
  va_list args;

  char buffer[128];

  va_start(args, format);
  vsprintf(buffer, format, args);
  sendMessage(buffer);
}

void sendBadPacket()
{
  // Tell the Pi that it sent us a packet with a bad
  // magic number.

  TPacket badPacket;
  badPacket.packetType = PACKET_TYPE_ERROR;
  badPacket.command = RESP_BAD_PACKET;
  sendResponse(&badPacket);

}

void sendBadChecksum()
{
  // Tell the Pi that it sent us a packet with a bad
  // checksum.

  TPacket badChecksum;
  badChecksum.packetType = PACKET_TYPE_ERROR;
  badChecksum.command = RESP_BAD_CHECKSUM;
  sendResponse(&badChecksum);
}

void sendBadCommand()
{
  // Tell the Pi that we don't understand its
  // command sent to us.

  TPacket badCommand;
  badCommand.packetType = PACKET_TYPE_ERROR;
  badCommand.command = RESP_BAD_COMMAND;
  sendResponse(&badCommand);

}

void sendBadResponse()
{
  TPacket badResponse;
  badResponse.packetType = PACKET_TYPE_ERROR;
  badResponse.command = RESP_BAD_RESPONSE;
  sendResponse(&badResponse);
}

void sendOK()
{
  TPacket okPacket;
  okPacket.packetType = PACKET_TYPE_RESPONSE;
  okPacket.command = RESP_OK;
  sendResponse(&okPacket);
}

void sendResponse(TPacket *packet)
{
  // Takes a packet, serializes it then sends it out
  // over the serial port.
  char buffer[PACKET_SIZE];
  int len;

  len = serialize(buffer, packet, sizeof(TPacket));
  writeSerial(buffer, len);
}


/*
   Setup and start codes for external interrupts and
   pullup resistors.

*/
// Enable pull up resistors on pins 2 and 3
void enablePullups()
{
  // Use bare-metal to enable the pull-up resistors on pins
  // 2 and 3. These are pins PD2 and PD3 respectively.
  // We set bits 2 and 3 in DDRD to 0 to make them inputs.
  DDRD &= 0b11110011;
  PORTD |= 0b00001100;
}

// Functions to be called by INT0 and INT1 ISRs.
void leftISR()
{
  switch (dir) {
    case FORWARD:
      //IRSensor();
      leftForwardTicks++;
      forwardDist += (unsigned long) ((float) leftForwardTicks / COUNTS_PER_REV * WHEEL_CIRC); //all ticks reset when user calls to stop()
      leftDist += (unsigned long) ((float) leftForwardTicks / COUNTS_PER_REV * WHEEL_CIRC); //all ticks reset when user calls to stop()
      break;
    case BACKWARD:
      //IRSensor();
      leftReverseTicks++;
      reverseDist += (unsigned long) ((float) leftReverseTicks / COUNTS_PER_REV * WHEEL_CIRC);
      leftDist -= (unsigned long) ((float) leftReverseTicks / COUNTS_PER_REV * WHEEL_CIRC);
      break;
    case LEFT:
      //IRSensor();
      leftReverseTicksTurns++;
      leftDist -=  (unsigned long) ((float) leftReverseTicksTurns / COUNTS_PER_REV * WHEEL_CIRC);
      break;
    case RIGHT:
      //IRSensor();
      leftForwardTicksTurns++;
      leftDist += (unsigned long) ((float) leftForwardTicksTurns / COUNTS_PER_REV * WHEEL_CIRC);
      break;
    default:
      break;
  }
  /*
     leftRevs = (unsigned long) ((float) leftTicks / COUNTS_PER_REV);
     forwardDist = (unsigned long) (leftRevs * WHEEL_CIRC);
  */
}

void rightISR()
{
  switch (dir) {
    case FORWARD:
      //IRSensor();
      rightForwardTicks++;
      rightDist += (unsigned long) ((float) rightForwardTicks / COUNTS_PER_REV * WHEEL_CIRC); //all ticks reset when user calls to stop()
      break;
    case BACKWARD:
      //IRSensor();
      rightReverseTicks++;
      rightDist -= (unsigned long) ((float) rightReverseTicks / COUNTS_PER_REV * WHEEL_CIRC);
      break;
    case LEFT:
      //IRSensor();
      rightForwardTicksTurns++;
      rightDist +=  (unsigned long) ((float) rightForwardTicksTurns / COUNTS_PER_REV * WHEEL_CIRC);
      break;
    case RIGHT:
      //IRSensor();
      rightReverseTicksTurns++;
      rightDist -= (unsigned long) ((float) rightReverseTicksTurns / COUNTS_PER_REV * WHEEL_CIRC);
      break;
    default:
      break;
  }

  /*
    rightRevs = (unsigned long) ((float) leftTicks / COUNTS_PER_REV);
    forwardDist = (unsigned long) (leftRevs * WHEEL_CIRC);
  */
}

// Set up the external interrupt pins INT0 and INT1
// for falling edge triggered. Use bare-metal.
void setupEINT()
{
  // Use bare-metal to configure pins 2 and 3 to be
  // falling edge triggered. Remember to enable
  // the INT0 and INT1 interrupts.

  // set INT0 and INT1 to falling edge
  EICRA = 0b00001010;

  // activate INT0 and INT1
  EIMSK = 0b00000011;
}

// Implement the external interrupt ISRs below.
// INT0 ISR should call leftISR while INT1 ISR
// should call rightISR.
ISR (INT0_vect)
{
  leftISR();
}

ISR (INT1_vect)
{
  rightISR();
}

ISR(TIMER0_COMPA_vect) {}
ISR(TIMER0_COMPB_vect) {}
ISR(TIMER1_COMPB_vect) {}
ISR(TIMER2_COMPA_vect) {}

// Implement INT0 and INT1 ISRs above.

/*
   Setup and start codes for serial communications

*/
// Set up the serial connection. For now we are using
// Arduino Wiring, you will replace this later
// with bare-metal code.
void setupSerial()
{
  // To replace later with bare-metal.
  //Serial.begin(9600);
  UBRR0L = 103;  // 9600 baud rate
  UBRR0H = 0;
  UCSR0C = 0b00000110;  // 8N1 asynchronous
  UCSR0A = 0;
  //Serial.begin(9600);
}

// Start the serial connection. For now we are using
// Arduino wiring and this function is empty. We will
// replace this later with bare-metal code.

void startSerial()
{
  // Empty for now. To be replaced with bare-metal code
  // later on.

  //Enable receiver & transmitter
  UCSR0B = 0b10011000;
}

// Read the serial port. Returns the read character in
// ch if available. Also returns TRUE if ch is valid.
// This will be replaced later with bare-metal code.

int readSerial(char *buffer)
{

  int count = 0;

  while (Serial.available())
    buffer[count++] = Serial.read();

  return count;
}

// Write to the serial port. Replaced later with
// bare-metal code

void writeSerial(const char *buffer, int len)
{
  Serial.write(buffer, len);
}

void setupMotors()
{
  /* Our motor set up is:
        A1IN - Pin 5, PD5, OC0B
        A2IN - Pin 6, PD6, OC0A
        B1IN - Pin 10, PB2, OC1B
        B2In - pIN 11, PB3, OC2A
  */
  pinMode(LF, OUTPUT);
  pinMode(LR, OUTPUT);
  pinMode(RF, OUTPUT);
  pinMode(RR, OUTPUT);
  /*DDRD |= 0b01100000;
    DDRB |= 0b00001100;

    FOR 0C0A/B:
    TCNT0 = 255;
    OCR0A = 0;
    OCR0B = 0;

    //For OC1B
    TCNT1 = 0;
    TCNT2 = 0;
    TCCR0A = 0b10100001;
    TCCR1A = 0b00100001;
    TCCR2A = 0b10000001;
    TIMSK0 |= 0b110;
    TIMSK1 |= 0b100;
    TIMSK2 |= 0b10;
    OCR0A = 0;
    OCR0B = 0;
    OCR1B = 0;
    OCR2A = 0;*/
}

// Start the PWM for Alex's motors.
// We will implement this later. For now it is
// blank.
void startMotors()
{
  //TCCR0B = 0b00000010;
  //TCCR1B = 0b00000010;
  //TCCR2B = 0b00000010;
}

// Convert percentages to PWM values
int pwmVal(float speed)
{
  if (speed < 0.0)
    speed = 0;

  if (speed > 100.0)
    speed = 100.0;

  return (int) ((speed / 100.0) * 255.0);
}

// Move Alex forward "dist" cm at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// move forward at half speed.
// Specifying a distance of 0 means Alex will
// continue moving forward indefinitely.
void forward()
{
  /*
    //to be compared in main loop (deltaDist = dist by user VS current total distance moved in dir)
    if(dist == 0)
    deltaDist = 9999999;
    else
    deltaDist = dist;
    //newDist=forwardDist + deltaDist;
  */



  analogWrite(LF, 0);
  analogWrite(RF, 0);
  analogWrite(LR, 170);
  analogWrite(RR, 170);
  delay(60);
  analogWrite(LF, 0);
  analogWrite(RF, 0);
  analogWrite(LR, 130);
  analogWrite(RR, 130);
  //}
  //else
  /*{
    analogWrite(5, 0);
    analogWrite(11, 0);
    analogWrite(6, 0);
    analogWrite(10, 0);
    }*/

  //OCR0A = 217;
  //OCR1B = 217;
  //OCR0B = 0;
  //OCR2A = 0;
}
void fforward()
{
  /*
    //to be compared in main loop (deltaDist = dist by user VS current total distance moved in dir)
    if(dist == 0)
    deltaDist = 9999999;
    else
    deltaDist = dist;
    //newDist=forwardDist + deltaDist;
  */



  //int val = pwmVal(speed);

  // For now we will ignore dist and move
  // forward indefinitely. We will fix this
  // in Week 9.

  // LF = Left forward pin, LR = Left reverse pin
  // RF = Right forward pin, RR = Right reverse pin
  // This will be replaced later with bare-metal code.
  //if (ultrasonic_distance > 5) {
  analogWrite(5, 250);
  analogWrite(11, 250);
  analogWrite(6, 0);
  analogWrite(10, 0);
  delay(1000);
  analogWrite(5, 0);
  analogWrite(11, 0);
  analogWrite(6, 0);
  analogWrite(10, 0);
  //dir = FORWARD;
  //}
  /*else
    {
    analogWrite(5, 0);
    analogWrite(11, 0);
    analogWrite(6, 0);
    analogWrite(10, 0);
    }*/

  //OCR0A = 217;
  //OCR1B = 217;
  //OCR0B = 0;
  //OCR2A = 0;
}
void reverse()
{
  //to be compared in main loop (deltaDist = dist by user VS current total distance moved in dir)
  analogWrite(LF, 180);
  analogWrite(RF, 180);
  analogWrite(LR, 0);
  analogWrite(RR, 0);
  delay(50);
  analogWrite(LF, 130);
  analogWrite(RF, 130);
  analogWrite(LR, 0);
  analogWrite(RR, 0);
  delay(500);
  analogWrite(LF, 0);
  analogWrite(RF, 0);
  analogWrite(LR, 0);
  analogWrite(RR, 0);
  //int val = pwmVal(speed);

  //OCR0B = 217;
  //OCR2A = 217;
  //OCR0A = 0;
  //OCR1B = 0;
}

unsigned long computeDeltaTicks(float ang)
{
  unsigned long ticks = (unsigned long) ((ang * alexCirc * COUNTS_PER_REV) / (360.0 * WHEEL_CIRC));
  return ticks;
}

void left()
{

  //dir = LEFT;

  //int val = pwmVal(speed);

  /*if (ang == 0)
    deltaTicks = 99999999;
    else
    deltaTicks = computeDeltaTicks(ang);
    targetTicks = leftReverseTicksTurns + deltaTicks;*/


  analogWrite(LF, 165);
  analogWrite(RF, 0);
  analogWrite(LR, 0);
  analogWrite(RR, 165);
  delay(50);
  analogWrite(LF, 140);
  analogWrite(RF, 0);
  analogWrite(LR, 0);
  analogWrite(RR, 140);
  delay(500);
  analogWrite(LF, 0);
  analogWrite(RF, 0);
  analogWrite(LR, 0);
  analogWrite(RR, 0);
  /*else
    {
    analogWrite(5, 0);
    analogWrite(11, 0);
    analogWrite(6, 0);
    analogWrite(10, 0);
    }*/


  //OCR0B = 230; //5 - LR
  //OCR1B = 100; //10 - RF
  //OCR0A = 0; //6 - LF
  //OCR2A = 0; //11 - RR
}

void right()
{
  //dir = RIGHT;

  //int val = pwmVal(speed);

  /*if (ang == 0)
    deltaTicks = 99999999;
    else
    delta,Ticks = computeDeltaTicks(ang);
    targetTicks = rightReverseTicksTurns + deltaTicks;*/

  //if (ultrasonic_distance > 5) {
  analogWrite(LF, 0);
  analogWrite(RF, 165);
  analogWrite(LR, 165);
  analogWrite(RR, 0);
  delay(50);
  analogWrite(LF, 0);
  analogWrite(RF, 140);
  analogWrite(LR, 140);
  analogWrite(RR, 0);
  delay(500);
  analogWrite(LF, 0);
  analogWrite(RF, 0);
  analogWrite(LR, 0);
  analogWrite(RR, 0);
  /*   delay(5000);
    analogWrite(5, 0);
    analogWrite(11, 0);
    analogWrite(6, 0);
    analogWrite(10, 0);*/
  //}
  /*else
    {
    analogWrite(5, 0);
    analogWrite(11, 0);
    analogWrite(6, 0);
    analogWrite(10, 0);
    }*/

  //OCR2A = 0;
  //OCR0A = 200;
  //OCR0B = 200;
  //OCR1B = 0;
}
void rearright()
{
  //dir = RIGHT;

  //int val = pwmVal(speed);

  /*if (ang == 0)
    deltaTicks = 99999999;
    else
    deltaTicks = computeDeltaTicks(ang);
    targetTicks = rightReverseTicksTurns + deltaTicks;*/


  analogWrite(5, 0);
  analogWrite(11, 0);
  analogWrite(6, 180);//RR
  analogWrite(10, 0);//LR
  //OCR2A = 0;
  //OCR0A = 200;
  //OCR0B = 200;
  //OCR1B = 0;
}
void rearleft()
{
  //dir = RIGHT;

  //int val = pwmVal(speed);

  /*if (ang == 0)
    deltaTicks = 99999999;
    else
    deltaTicks = computeDeltaTicks(ang);
    targetTicks = rightReverseTicksTurns + deltaTicks;*/


  analogWrite(5, 0);
  analogWrite(11, 0);
  analogWrite(6, 0);//RR
  analogWrite(10, 180);//LR
  //OCR2A = 0;
  //OCR0A = 200;
  //OCR0B = 200;
  //OCR1B = 0;
}
// Stop Alex. To replace with bare-metal code later.
void stop()
{
  dir = STOP;
  OCR0A = 0;
  OCR1B = 0;
  OCR0B = 0;
  OCR2A = 0;

  clearCounters();
}

/*
   Alex's setup and run codes

*/

// Clears all our counters
void clearCounters()
{
  leftForwardTicks = 0;
  leftReverseTicks = 0;
  rightForwardTicks = 0;
  rightReverseTicks = 0;

  leftForwardTicksTurns = 0;
  leftReverseTicksTurns = 0;
  rightForwardTicksTurns = 0;
  rightReverseTicksTurns = 0;

  leftRevs = 0;
  rightRevs = 0;

  forwardDist = 0;
  reverseDist = 0;
}

// Clears one particular counter
void clearOneCounter(int which)
{
  clearCounters();
}
// Intialize Vincet's internal states

void initializeState()
{
  clearCounters();
}

void handleCommand(TPacket *command)
{
  switch (command->command)
  {
    // For movement commands, param[0] = distance, param[1] = speed.
    case COMMAND_FORWARD:
      sendOK();
      ultrasonicCheck();
      proxyIRcheck();
      //colread();
      sendUltrasonicIRinfo();
      forward();
      break;
    case COMMAND_REVERSE:
      sendOK();

      ultrasonicCheck();
      proxyIRcheck();
      //colread();
      sendUltrasonicIRinfo();
      reverse();
      break;
    case COMMAND_TURN_LEFT:
      sendOK();

      ultrasonicCheck();
      proxyIRcheck();
      //colread();
      sendUltrasonicIRinfo();
      left();
      break;
    case COMMAND_TURN_RIGHT:
      sendOK();

      ultrasonicCheck();
      proxyIRcheck();
      //colread();
      sendUltrasonicIRinfo();
      right();
      break;
    case COMMAND_STOP:
      sendOK();
      stop();
      ultrasonicCheck();
      proxyIRcheck();
      //colread();
      sendUltrasonicIRinfo();
      break;
    case COMMAND_REV_LEFT:
      sendOK();

      ultrasonicCheck();
      proxyIRcheck();
      //colread();
      sendUltrasonicIRinfo();
      rearleft();
      break;
    case COMMAND_REV_RIGHT:
      sendOK();

      ultrasonicCheck();
      proxyIRcheck();
      //colread();
      sendUltrasonicIRinfo();
      rearright();
      break;
    case COMMAND_COLOUR:
      sendOK();
      colread();
      sendUltrasonicIRinfo();
      //findColour();
      //sendColour();
      break;
    case COMMAND_GET_STATS:
      //sendStatus();
      sendOK();
      fforward();
      ultrasonicCheck();
      proxyIRcheck();
      //colread();
      sendUltrasonicIRinfo();
      break;
      break;
    case COMMAND_CLEAR_STATS:
      clearOneCounter(command->params[0]); //clears all values
      sendOK();
      break;
    default:
      sendBadCommand();
  }
}



void waitForHello()
{
  int exit = 0;

  while (!exit)//!exit
  {
    TPacket hello;
    TResult result;

    do
    {
      result = readPacket(&hello);
    } while (result == PACKET_INCOMPLETE);

    if (result == PACKET_OK)
    {
      if (hello.packetType == PACKET_TYPE_HELLO)
      {
        sendOK();
        exit = 1;
      }
      else
        sendBadResponse();
    }
    else if (result == PACKET_BAD)
      sendBadPacket();
    else if (result == PACKET_CHECKSUM_BAD)
      sendBadChecksum();

  } // !exit
}

void setup() {
  // put your setup code here, to run once:
  alexDiagonal = sqrt((ALEX_LENGTH * ALEX_LENGTH) + (ALEX_BREADTH * ALEX_BREADTH));
  alexCirc = PI * alexDiagonal;

  cli();
  setupPowerSaving();
  setupEINT();
  setupSerial();
  setupIR();
  setupUltrasonic();
  colsetup();
  //setupColour();
  startSerial();
  setupMotors();
  startMotors();
  enablePullups();
  initializeState();

  sei();

  waitForHello();
}

void handleParams(TPacket *packet) {
  ultrasonicCheck();
  proxyIRcheck();
  //colread();
  sendUltrasonicIRinfo();
}

void handlePacket(TPacket *packet)
{
  switch (packet->packetType)
  {
    case PACKET_TYPE_COMMAND:
      handleCommand(packet);
      //handleParams(packet);
      break;

    case PACKET_TYPE_RESPONSE:
      break;

    case PACKET_TYPE_ERROR:
      break;

    case PACKET_TYPE_MESSAGE:
      break;

    case PACKET_TYPE_HELLO:
      break;
  }
}
//continuously ping for ir and ultrasonic
void loop() {
  //forward();
  // put your main code here, to run repeatedly:
  TPacket recvPacket; // This holds commands from the Pi

  TResult result = readPacket(&recvPacket);

  if (result == PACKET_OK) {
    //sendStatus();
    //ultrasonicCheck();
    //sendUltrasonicDist();
    handlePacket(&recvPacket);

  }
  else if (result == PACKET_BAD)
    sendBadPacket();
  else if (result == PACKET_CHECKSUM_BAD)
    sendBadChecksum();

  if (dir == STOP) {
    putArduinoToIdle(); //save power
  }



  //prev_ultrasonic_dist = ultrasonic_distance;
  //sendUltrasonicDist();
  //Serial.println(prev_ultrasonic_dist);
  /*looper++;
    if(looper>=100){
    ultrasonicCheck();
    looper=0;
    }*/

  /*if (ultrasonic_distance - prev_ultrasonic_dist >= 1 || prev_ultrasonic_dist - ultrasonic_distance >= 1) {
    //update ultrasonic distance value:
    prev_ultrasonic_dist=ultrasonic_distance;
    //send the info:
    sendUltrasonicDist();
    //sendStatus();
    }*/
  //REMOVED DISTANCE - BOT ONLY STOPS WHEN STOP() IS CALLED:
  //Do the checks continously:
  /*proxyIRcheck(); //checks if bot is too close to side walls or back wall
    ultrasonicCheck(); //checks if bot is too close to front obstacle
    //stop when too close:
    if (ultrasonic_distance <= 3) {
    stop();
    findColour();
    sendColour();
    sendMessage("Warning: Too close to front");
    sendMessage("Advice: Key in colour command.");
    }
    if (LF_IR_Val == 0 || LR_IR_Val == 0) {
    //stop();
    sendMessage("Warning: Too close to left");
    sendMessage("Advice: Move right");
    }
    if (RF_IR_Val == 0 || RR_IR_Val == 0) {
    //stop();
    sendMessage("Warning: Too close to right");
    sendMessage("Advice: Move left");
    }
    if (Rear_IR_Val == 0) {
    //stop();
    sendMessage("Warning: Too close to rear");
    sendMessage("Advice: Move forward");
    }*/

  //error with IR
  /*if ((prev_LF_IR_Val != LF_IR_Val) || (prev_LR_IR_Val != LR_IR_Val) || (prev_RF_IR_Val !=RF_IR_Val) || (prev_RR_IR_Val != RR_IR_Val) || (prev_Rear_IR_Val != Rear_IR_Val) {
    //update IR values:
    prev_LF_IR_Val = LF_IR_Val;
    prev_LR_IR_Val = LR_IR_Val;
    prev_RF_IR_Val = RF_IR_Val;
    prev_RR_IR_Val = RR_IR_Val;
    prev_Rear_IR_Val = Rear_IR_Val;
    //send the info
    sendIR();
    }*/
  /*
    if (deltaDist > 0)
    {
    if (dir == FORWARD) {
    if (forwardDist > newDist) {
      deltaDist = 0;
      newDist = 0;
      stop();
    }
    }
    else if (dir == BACKWARD) {
    if (reverseDist > newDist) {
      deltaDist = 0;
      newDist = 0;
      stop();
    }
    }
    else if (dir == STOP) {
    deltaDist = 0;
    newDist = 0;
    stop();
    }
    }

    if (deltaTicks > 0)
    {
    if (dir == LEFT)
    {
    //while (leftReverseTicksTurns==computeDeltaTicks(30) || leftReverseTicksTurns==computeDeltaTicks(60)) {
    //delay(1000);
    //}

    if (leftReverseTicksTurns >= targetTicks) {
      deltaTicks = 0;
      targetTicks = 0;
      stop();
    }
    }
    else if (dir == RIGHT)
    {
    //while (rightReverseTicksTurns==computeDeltaTicks(30) || rightReverseTicksTurns==computeDeltaTicks(60)) {
    //delay(1000);
    //}
    if (rightReverseTicksTurns >= targetTicks) {
      deltaTicks = 0;
      targetTicks = 0;
      stop();
    }
    }
    else if (dir == STOP) {
    deltaTicks = 0;
    targetTicks = 0;
    stop();
    }
    }*/

}

