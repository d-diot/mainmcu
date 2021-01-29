/*
 * This is the sketch for the ATMega328P MCU or the Arduino Nano of the d-diot-board-4B
 */

#include <Arduino.h>

// ########################### CONFIG ##########################################

// General definitions
#define ENABLE_STATUS_LED //Uncomment to enable the status LED
//#define DEBUG             //Uncomment to enable the debug messages in the serial output
//#define BB_DEBUG          //Uncomment to enable the buck and boost debug messages in the serial output
//#define DEBUG_DELAY 500   //Uncomment to slow down the code speed in debug mode

// Status LED brightness (0-255)
#ifdef ENABLE_STATUS_LED
int StautsLedBrightness = 120;
#endif

// Boost Converter parameters
int BoostTargetVoltage = 10000; //Desired output voltage of the boost converter (Millivolts)
long BoostR1 = 10000;           //R1 value in Ohm (feedback circuit)
long BoostR2 = 3300;            //R2 value in Ohm (feedback circuit)
int BoostPwm = 0;               //Initial value of boost PWM (0 = off)
int BoostMaxVoltage = 20000;    //Maximum voltage allowed by the circuit design (Millivolts)
int BoostMaxPwm = 200;          //Maximum PWM value (safety features)

// Buck converter parameters
int BuckTargetVoltage = 2500; //Desired output voltage of the buck converter (Millivolts)
long BuckR1 = 0;              //R1 value in Ohm (feedback circuit, 0 = no feedback circuit)
long BuckR2 = 0;              //R2 value in Ohm (feedback circuit, 0 = no feedback circuit)
int BuckPwm = 255;            //Initial value of buck PWM (255 = off)
int BuckMinVoltage = 1500;    //Minimum voltage allowed by the circuit design

// Vcc sampling frequency: 0-65535. 0 = check Vcc at every loop cycle, 10 = check every 100 loop cycles
unsigned int VccSamplingFreq = 10000;

// Power line opening delay time
unsigned long power_line_wait_time = 500; // Time to wait after opening a power line. Necessary to avoid peak of current adsorption

// ####################### END OF CONFIG ###########################################

// ########################### INIT ################################################

// PIN definition
int BoostPin = 3;          //Digital pin D3 for boost PWM signal
int BuckPin = 11;          //Digital pin D11 for buck PWM signal
int PowerlinePin = 4;      //Pin that controls the buck and oost converter power line
int StatusLedPin = 9;      //Pin that control the status LED
int BoostFeedbackPin = A6; //The boost feedback input is A6 (pin 20)
int BuckFeedbackPin = A7;  //The buck feedback input is A7 (pin 21)

long Vcc;
bool PowerlineStatus;
long BoostAcutalVoltage;
long BuckAcutalVoltage;
unsigned int LoopCounter = 0;

// ########################## END OF INIT ##########################################

// ########################### CUSTOM FUNCTIONS ####################################

long readVcc()
{
// Read 1.1V reference against AVcc
// set the reference to Vcc and the measurement to the internal 1.1V reference
#if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#elif defined(__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
  ADMUX = _BV(MUX5) | _BV(MUX0);
#elif defined(__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
  ADMUX = _BV(MUX3) | _BV(MUX2);
#else
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#endif

  delay(2);            // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA, ADSC))
    ; // measuring

  uint8_t low = ADCL;  // must read ADCL first - it then locks ADCH
  uint8_t high = ADCH; // unlocks both

  long result = (high << 8) | low;

  result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  return result;              // Vcc in millivolts
}

long read_voltage(int PinNumber, long R1, long R2)
{
  long vout = analogRead(PinNumber) * Vcc / 1023;
  if (R1 > 0 && R2 > 0)
  {
    vout = vout * (R1 + R2) / R2;
  }
#ifdef BB_DEBUG
  Serial.print("Vout: ");
  Serial.print(vout);
  Serial.println(" mV");
#endif
  return vout;
}

void check_power_line_status()
{
  PowerlineStatus = !digitalRead(PowerlinePin);
#ifdef DEBUG
  Serial.print("Power line status: ");
  Serial.println(PowerlineStatus);
#endif
}

void open_power_line()
{
  digitalWrite(PowerlinePin, LOW);
  PowerlineStatus = true;
  delay(power_line_wait_time);
#ifdef DEBUG
  Serial.println("Boost power line opened");
#endif
}

void close_power_line()
{
  digitalWrite(PowerlinePin, HIGH);
  PowerlineStatus = false;
  delay(power_line_wait_time);
#ifdef DEBUG
  Serial.println("Boost power line opened");
#endif
}

void update_boost_converter()
{
  //If the target value is HIGHER than the real value, we increase PWM width
#ifdef BB_DEBUG
  Serial.println("Boost converter");
#endif
  if (BoostTargetVoltage > BoostMaxVoltage || !PowerlineStatus)
  {
    BoostPwm = 0;
    digitalWrite(BoostPin, LOW);
#ifdef BB_DEBUG
    Serial.println("Target voltage to high or power line off: shutdown!");
#endif
    return;
  }
  BoostAcutalVoltage = read_voltage(BoostFeedbackPin, BoostR1, BoostR2);
  if (BoostTargetVoltage > BoostAcutalVoltage)
  {
    BoostPwm = BoostPwm + 1;
    BoostPwm = constrain(BoostPwm, 0, BoostMaxPwm);
  }
  else if (BoostTargetVoltage < BoostAcutalVoltage)
  {
    BoostPwm = BoostPwm - 1;
    BoostPwm = constrain(BoostPwm, 0, BoostMaxPwm);
  }
#ifdef BB_DEBUG
  Serial.print("PWM: ");
  Serial.println(BoostPwm);
#endif
  analogWrite(BoostPin, BoostPwm);
}

void update_buck_converter()
{
  //If the target value is HIGHER than the real value, we decrease PWM width (p-mos, reverse logic)
#ifdef BB_DEBUG
  Serial.println("Buck converter");
#endif
  if (BuckTargetVoltage < BuckMinVoltage || !PowerlineStatus)
  {
    BuckPwm = 255;
    digitalWrite(BuckPin, HIGH);
#ifdef BB_DEBUG
    Serial.println("Target voltage to low or power line off: shutdown!");
#endif
    return;
  }
  BuckAcutalVoltage = read_voltage(BuckFeedbackPin, BuckR1, BuckR2);
  if (BuckTargetVoltage > BuckAcutalVoltage)
  {
    BuckPwm = BuckPwm - 1;
    BuckPwm = constrain(BuckPwm, 0, 255);
  }
  else if (BuckTargetVoltage < BuckAcutalVoltage)
  {
    BuckPwm = BuckPwm + 1;
    BuckPwm = constrain(BuckPwm, 0, 255);
  }
#ifdef BB_DEBUG
  Serial.print("PWM: ");
  Serial.println(BuckPwm);
#endif
  analogWrite(BuckPin, BuckPwm);
}

// ########################### END OF CUSTOM FUNCTIONS #################################

// ##################################### SETUP #########################################

void setup()
{
  pinMode(BuckFeedbackPin, INPUT);
  pinMode(BoostFeedbackPin, INPUT);
  pinMode(BoostPin, OUTPUT);
  pinMode(BuckPin, OUTPUT);
  pinMode(StatusLedPin, OUTPUT);
  pinMode(PowerlinePin, OUTPUT);
  // Status LED
#ifdef ENABLE_STATUS_LED
  analogWrite(StatusLedPin, StautsLedBrightness);
#endif
#ifndef ENABLE_STATUS_LED
  digitalWrite(StatusLedPin, LOW);
#endif
// Serial initialization
#ifdef DEBUG
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
#endif
  // Power line opening
  open_power_line();
  // pin 3 and 11 PWM frequency of 31372.55 Hz
  TCCR2B = TCCR2B & B11111000 | B00000001;
  // Initialize Buck and Boost converter
  analogWrite(BoostPin, BoostPwm);
  analogWrite(BuckPin, BuckPwm);
  // Vcc read
  Vcc = readVcc();
#ifdef DEBUG
  Serial.print("Vcc: ");
  Serial.print(Vcc);
  Serial.println(" mV");
#endif
}

// ##################################### END OF SETUP #####################################

// ##################################### LOOP #############################################

void loop()
{
  // Check Vcc if needed
  LoopCounter++;
  if (LoopCounter >= VccSamplingFreq)
  {
    Vcc = readVcc();
#ifdef DEBUG
    Serial.print("Loop Number: ");
    Serial.print(LoopCounter);
    Serial.print(". Reading Vcc... ");
    Serial.print(Vcc);
    Serial.println(" mV");
#endif
    LoopCounter = 0;
  }
  // Check boost and boost converter Vout and update PWM if necessary
  update_boost_converter();
  update_buck_converter();
#ifdef DEBUG_DELAY
  delay(DEBUG_DELAY);
#endif
}