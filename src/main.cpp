/*
 * This is the sketch for the ATMega328P MCU of the d-diot board v.3.0
 */

#include <Arduino.h>

// ########################### CONFIG ##########################################
// Configuration
#define ENABLE_BOOST
#define ENABLE_BUCK
//#define DEBUG

// Boost Converter parameters
int BoostTargetVoltage = 10000; //Desired output voltage of the boost converter (Millivolts)
long BoostR1 = 68000;           //R1 value in Ohm (feedback circuit)
long BoostR2 = 47000;           //R2 value in Ohm (feedback circuit)
int BoostFeedbackPin = A6;      //The boost feedback input is A6 (pin 20)
int BoostPin = 3;               //Digital pin D3 for boost PWM signal
int BoostPwm = 0;               //Initial value of PWM boost width
int BoostMaxVoltage = 12000;    //Maximum voltage for the board safety

// Buck converter parameters
int BuckTargetVoltage = 1800; //Desired output voltage of the buck converter (Millivolts)
long BuckR1 = 0;              //R1 value in Ohm (feedback circuit)
long BuckR2 = 0;              //R2 value in Ohm (feedback circuit)
int BuckFeedbackPin = A7;     //The buck feedback input is A7 (pin 21)
int BuckPin = 11;             //Digital pin D11 for buck PWM signal
int BuckPwm = 255;            //Initial value of PWM buck width
int BuckMinVoltage = 1500;    //Minimum voltage for the board safety

// ####################### END OF CONFIG ###########################################

// ########################### GLOBAL ##############################################

long Vcc;

// ########################## END OF GLOBAL ########################################

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
#ifdef DEBUG
  Serial.print("Vout: ");
  Serial.print(vout);
  Serial.println(" mV");
#endif
  return vout;
}

// ########################### END OF CUSTOM FUNCTIONS #################################

// ##################################### SETUP #########################################

void setup()
{
  pinMode(BoostFeedbackPin, INPUT);
  pinMode(BoostPin, OUTPUT);
  pinMode(BuckFeedbackPin, INPUT);
  pinMode(BuckPin, OUTPUT);
  TCCR2B = TCCR2B & B11111000 | B00000001; // pin 3 and 11 PWM frequency of 31372.55 Hz
  analogWrite(BoostPin, BoostPwm);
  analogWrite(BuckPin, BuckPwm);
  Vcc = readVcc();
#ifdef DEBUG
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
  Serial.print("Vcc: ");
  Serial.print(Vcc);
  Serial.println(" mV");
#endif
}

// ##################################### END OF SETUP #####################################

// ##################################### LOOP #############################################

void loop()
{
#ifdef ENABLE_BOOST
//If the desired value is HIGHER than the real value, we increase PWM width
#ifdef DEBUG
  Serial.println("Boost converter");
#endif
  long BoostAcutalVoltage = read_voltage(BoostFeedbackPin, BoostR1, BoostR2);
  if (BoostTargetVoltage > BoostAcutalVoltage)
  {
    BoostPwm = BoostPwm + 1;
    BoostPwm = constrain(BoostPwm, 1, 254);
  }
  else if (BoostTargetVoltage < BoostAcutalVoltage)
  {
    BoostPwm = BoostPwm - 1;
    BoostPwm = constrain(BoostPwm, 1, 254);
  }
  if (BoostTargetVoltage > BoostMaxVoltage)
  {
    BoostPwm = 0;
#ifdef DEBUG
    Serial.println("Target voltage to high: shutdown!");
#endif
  }
#ifdef DEBUG
  Serial.print("PWM: ");
  Serial.println(BoostPwm);
#endif
  analogWrite(BoostPin, BoostPwm);
#endif

#ifdef ENABLE_BUCK
//If the desired value is HIGHER than the real value, we increase PWM width
#ifdef DEBUG
  Serial.println("Buck converter");
#endif
  long BuckAcutalVoltage = read_voltage(BuckFeedbackPin, BuckR1, BuckR2);
  if (BuckTargetVoltage > BuckAcutalVoltage)
  {
    BuckPwm = BuckPwm - 1;
    BuckPwm = constrain(BuckPwm, 1, 254);
  }
  else if (BuckTargetVoltage < BuckAcutalVoltage)
  {
    BuckPwm = BuckPwm + 1;
    BuckPwm = constrain(BuckPwm, 1, 254);
  }
  if (BuckTargetVoltage < BuckMinVoltage)
  {
    BuckPwm = 255;
#ifdef DEBUG
    Serial.println("Target voltage to low: shutdown!");
#endif
  }
#ifdef DEBUG
  Serial.print("PWM: ");
  Serial.println(BuckPwm);
#endif
  analogWrite(BuckPin, BuckPwm);
#endif
}