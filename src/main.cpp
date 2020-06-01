/*
 * This is the sketch for the ATMega328P MCU of the d-diot board v.3.0
 */

#include <Arduino.h>

// ########################### CONFIG ##########################################
// Definition
#define ENV_PLATFORMIO
//#define EXTERNAL_BOOST
//#define EXTERNAL_BUCK
//#define DEBUG

// PIN definition
int ButtonPin = 2;           //Button PIN: is the same for physical button and TTP223 capacitive touch switch with A closed and B open (see https://www.hackster.io/najad/how-to-use-a-ttp223-based-touch-switch-a04f7d).
int BoostPin = 3;            //Digital pin D3 for boost PWM signal
int PiPowerOffPin = 4;       //Pin connected to the Raspberry Pi poweroff signal PIN (GPIO6)
int PiShutdownPin = 7;       //Pin connected to the Raspberry Pi shutdown PIN (GPIO5)
int PiPowerlinePin = 8;      //Pin that controls the main power line (Raspberry Pi)
int BuckPin = 11;            //Digital pin D11 for buck PWM signal
int BoostPowerlinePin = 12;  //Pin that controls the boost converter power line
int BuckPowerlinePin = A0;   //Pin that controls the buck converter power line
int RFLinkPowerlinePin = A1; //Pin that controls the ATMega2560 (RFLink) power line
int VPowerlinePin = A2;      //Pin that controls the 3.3V (AMS117) power line
int OnPowerPin = A3;         //Pin that controls the on / off state of the system when the board is powered for the first time
int BoostFeedbackPin = A6;   //The boost feedback input is A6 (pin 20)
int BuckFeedbackPin = A7;    //The buck feedback input is A7 (pin 21)

// Boost Converter parameters
int BoostTargetVoltage = 10000; //Desired output voltage of the boost converter (Millivolts)
long BoostR1 = 68000;           //R1 value in Ohm (feedback circuit)
long BoostR2 = 47000;           //R2 value in Ohm (feedback circuit)
int BoostPwm = 0;               //Initial value of PWM boost width
int BoostMaxVoltage = 12000;    //Maximum voltage for the board safety
int BoostMaxPwm = 200;          //Maximum PWM value

// Buck converter parameters
int BuckTargetVoltage = 1700; //Desired output voltage of the buck converter (Millivolts)
long BuckR1 = 0;              //R1 value in Ohm (feedback circuit)
long BuckR2 = 0;              //R2 value in Ohm (feedback circuit)
int BuckPwm = 255;            //Initial value of PWM buck width
int BuckMinVoltage = 1500;    //Minimum voltage for the board safety

// Button parameters
int DebounceInterval = 5; //Debounce interval in milliseconds

// Pi Shutdown pin parameters
uint8_t pi_shutdown_pin_press_time = 100; // Time in milliseconds to keep the pi shutdown pin LOW

// Pi power state monitoring time
uint16_t pi_poweroff_bounce_time = 1500; // Time of active monitoring in milliseconds to wait before confirming the status of the on / off state of the Pi

// ####################### END OF CONFIG ###########################################

// ########################### INIT ################################################

long Vcc;
bool pi_poweroff_pin_status;
bool pi_shutdown_pin_activated = false;
bool monitor_pi_status = false;
bool pi_status;
bool PiPowerlineStatus;
bool BoostPowerlineStatus;
bool BuckPowerlineStatus;
bool RFLinkPowerlineStatus;
bool VPowerlineStatus;
unsigned long last_pi_poweroff_pin_change_time = 0;
unsigned long current_time_millis;
unsigned long button_press_millis = 0;
unsigned long pi_shutdown_pin_press_start = 0;
long BoostAcutalVoltage;
long BuckAcutalVoltage;
#ifdef ENV_PLATFORMIO
#include <Bounce2.h>
#endif
#ifndef ENV_PLATFORMIO
#include "libraries/Bounce2/Bounce2.cpp"
#include "libraries/Bounce2/Bounce2.h"
#endif
Bounce debouncer = Bounce();

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
#ifdef DEBUG
  Serial.print("Vout: ");
  Serial.print(vout);
  Serial.println(" mV");
#endif
  return vout;
}

void check_power_lines_status()
{
  PiPowerlineStatus = !digitalRead(PiPowerlinePin);
  BoostPowerlineStatus = !digitalRead(BoostPowerlinePin);
  BuckPowerlineStatus = !digitalRead(BuckPowerlineStatus);
  RFLinkPowerlineStatus = !digitalRead(RFLinkPowerlineStatus);
  VPowerlineStatus = !digitalRead(VPowerlineStatus);
#ifdef DEBUG
  Serial.print("Pi power line status: ");
  Serial.println(PiPowerlineStatus);
  Serial.print("Boost power line status: ");
  Serial.println(BoostPowerlineStatus);
  Serial.print("Buck power line status: ");
  Serial.println(BuckPowerlineStatus);
  Serial.print("RFLink power line status: ");
  Serial.println(RFLinkPowerlineStatus);
  Serial.print("3.3V and 5V main power lines status: ");
  Serial.println(VPowerlineStatus);
#endif
}

void update_pi_status()
{
  check_power_lines_status();
  bool current_poweroff_pin_status = digitalRead(PiPowerOffPin);
  if (PiPowerlineStatus && current_poweroff_pin_status)
  {
    pi_status = true;
#ifdef DEBUG
    Serial.print("Pi is ON: power line and poweroff pin in agreement");
#endif
  }
  else if (!PiPowerlineStatus && !current_poweroff_pin_status)
  {
    pi_status = false;
#ifdef DEBUG
    Serial.print("Pi is OFF: power line and poweroff pin in agreement");
#endif
  }
  else if (PiPowerlineStatus && !current_poweroff_pin_status)
  {
    pi_status = true;
#ifdef DEBUG
    Serial.print("Pi is ON: power line is ON, but the Pi is signaling a power OFF state: probably a poweroff pin bounce during startup");
#endif
  }
  else if (!PiPowerlineStatus && current_poweroff_pin_status)
  {
    pi_status = true;
#ifdef DEBUG
    Serial.print("Pi is ON: power line is OFF, but the Pi is signaling a powero ON state probably bypass jumper is closed");
#endif
  }
}

void update_boost_converter()
{
#ifndef EXTERNAL_BOOST
  //If the desired value is HIGHER than the real value, we increase PWM width
#ifdef DEBUG
  Serial.println("Boost converter");
#endif
  if (BoostTargetVoltage > BoostMaxVoltage || !BoostPowerlineStatus)
  {
    BoostPwm = 0;
    digitalWrite(BoostPin, LOW);
#ifdef DEBUG
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
#ifdef DEBUG
  Serial.print("PWM: ");
  Serial.println(BoostPwm);
#endif
  analogWrite(BoostPin, BoostPwm);
#endif
#ifdef EXTERNAL_BOOST
  if (digitalRead(BoostPin))
  {
    digitalWrite(BoostPin, LOW);
  }
#endif
}

void update_buck_converter()
{
#ifndef EXTERNAL_BUCK
  //If the desired value is HIGHER than the real value, we decrease PWM width (p-mos, reverse logic)
#ifdef DEBUG
  Serial.println("Buck converter");
#endif
  if (BuckTargetVoltage < BuckMinVoltage || !BuckPowerlineStatus)
  {
    BuckPwm = 255;
    digitalWrite(BuckPin, HIGH);
#ifdef DEBUG
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
#ifdef DEBUG
  Serial.print("PWM: ");
  Serial.println(BuckPwm);
#endif
  analogWrite(BuckPin, BuckPwm);
#endif
#ifdef EXTERNAL_BUCK
  if (!digitalRead(BuckPin))
  {
    digitalWrite(BuckPin, HIGH);
  }
#endif
}

void activate_pi_shutdown_pin()
{
  if (!pi_shutdown_pin_activated)
  {
    digitalWrite(PiShutdownPin, HIGH);
    pi_shutdown_pin_press_start = millis();
    pi_shutdown_pin_activated = true;
#ifdef DEBUG
    Serial.println("Pi shutdown pin activated");
#endif
  }
}

void release_pi_shutdown_pin()
{
  digitalWrite(PiShutdownPin, LOW);
  pi_shutdown_pin_press_start = 0;
  pi_shutdown_pin_activated = false;
#ifdef DEBUG
  Serial.println("Pi shutdown pin released");
#endif
}

void open_all_powerlines()
{
  check_power_lines_status();
  if (!PiPowerlineStatus)
  {
    digitalWrite(PiPowerlinePin, LOW);
  }
  if (!BoostPowerlineStatus)
  {
    digitalWrite(BoostPowerlinePin, LOW);
  }
  if (!BuckPowerlineStatus)
  {
    digitalWrite(BuckPowerlinePin, LOW);
  }
  if (!RFLinkPowerlineStatus)
  {
    digitalWrite(RFLinkPowerlinePin, LOW);
  }
  if (!VPowerlineStatus)
  {
    digitalWrite(VPowerlinePin, LOW);
  }
  PiPowerlineStatus = true;
  BoostPowerlineStatus = true;
  BuckPowerlineStatus = true;
  RFLinkPowerlineStatus = true;
  VPowerlineStatus = true;
#ifdef DEBUG
  Serial.println("All power lines opened");
#endif
}

void close_all_powerlines()
{
  check_power_lines_status();
  if (PiPowerlineStatus)
  {
    digitalWrite(PiPowerlinePin, HIGH);
  }
  if (BoostPowerlineStatus)
  {
    digitalWrite(BoostPowerlinePin, HIGH);
  }
  if (BuckPowerlineStatus)
  {
    digitalWrite(BuckPowerlinePin, HIGH);
  }
  if (RFLinkPowerlineStatus)
  {
    digitalWrite(RFLinkPowerlinePin, HIGH);
  }
  if (VPowerlineStatus)
  {
    digitalWrite(VPowerlinePin, HIGH);
  }
  PiPowerlineStatus = false;
  BoostPowerlineStatus = false;
  BuckPowerlineStatus = false;
  RFLinkPowerlineStatus = false;
  VPowerlineStatus = false;
#ifdef DEBUG
  Serial.println("All power lines closed");
#endif
}

void on_button_release()
{
  // Short press detection (50 -350 ms)
  if (current_time_millis - button_press_millis >= 50 && current_time_millis - button_press_millis <= 350)
  {
#ifdef DEBUG
    Serial.println("Button click type: short");
#endif
    update_pi_status();
    pi_status ? activate_pi_shutdown_pin() : open_all_powerlines();
  }
  // Long press detection (> 3000 ms)
  else if (current_time_millis - button_press_millis >= 3000)
  {
#ifdef DEBUG
    Serial.println("Button click type: long");
#endif
    update_pi_status();
    pi_status ? close_all_powerlines() : open_all_powerlines();
  }
  else
  {
#ifdef DEBUG
    Serial.println("Button click type: unknown, do nothing");
#endif
    return;
  }
}

// ########################### END OF CUSTOM FUNCTIONS #################################

// ##################################### SETUP #########################################

void setup()
{
  pinMode(PiPowerOffPin, INPUT_PULLUP);
  pinMode(OnPowerPin, INPUT_PULLUP);
  pinMode(ButtonPin, INPUT_PULLUP);
  pinMode(BuckFeedbackPin, INPUT);
  pinMode(BoostFeedbackPin, INPUT);
  pinMode(PiPowerlinePin, OUTPUT);
  pinMode(BoostPin, OUTPUT);
  pinMode(BuckPin, OUTPUT);
  pinMode(PiShutdownPin, OUTPUT);
  pinMode(BoostPowerlinePin, OUTPUT);
  pinMode(BuckPowerlinePin, OUTPUT);
  pinMode(RFLinkPowerlinePin, OUTPUT);
  pinMode(VPowerlinePin, OUTPUT);
  TCCR2B = TCCR2B & B11111000 | B00000001; // pin 3 and 11 PWM frequency of 31372.55 Hz
  analogWrite(BoostPin, BoostPwm);
  analogWrite(BuckPin, BuckPwm);
  // Check the OnPowerPin status and power on or off the hub accordingly
  digitalRead(OnPowerPin) ? close_all_powerlines() : open_all_powerlines();
  // Read the Pi poweroff pin status
  pi_poweroff_pin_status = digitalRead(PiPowerOffPin);
  last_pi_poweroff_pin_change_time = millis();
  // Switch on all the power lines if the Pi is running
  if (pi_poweroff_pin_status)
  {
    open_all_powerlines();
  }
  // Deactivate the Pi shutdown PIN
  digitalWrite(PiShutdownPin, LOW);
  // Update Pi status and monitor
  update_pi_status();
  monitor_pi_status = true;
  Vcc = readVcc();
  debouncer.attach(ButtonPin, INPUT_PULLUP);
  debouncer.interval(DebounceInterval);
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
  // Timer Update
  current_time_millis = millis();

  // Release Pi shutdown PIN if necessary
  if (pi_shutdown_pin_activated)
  {
    if (current_time_millis - pi_shutdown_pin_press_start >= pi_shutdown_pin_press_time)
    {
      release_pi_shutdown_pin();
    }
  }

  // Check the Pi Poweroff Pin
  if (digitalRead(PiPowerOffPin) != pi_poweroff_pin_status)
  {
    pi_poweroff_pin_status = !pi_poweroff_pin_status;
    last_pi_poweroff_pin_change_time = current_time_millis;
    monitor_pi_status = true;
#ifdef DEBUG
    Serial.println("Pi poweroff pin status change detected: start monitoring");
#endif
  }

  // Monitor the pi status and at the end turn it on or off
  if (monitor_pi_status)
  {
    update_pi_status();
    if (current_time_millis - last_pi_poweroff_pin_change_time >= pi_poweroff_bounce_time)
    {
      monitor_pi_status = false;
      last_pi_poweroff_pin_change_time = 0;
#ifdef DEBUG
      Serial.println("Monitoring finished: take action");
#endif
      if (pi_status)
      {
        open_all_powerlines();
      }
      else
      {
        close_all_powerlines();
      }
    }
  }

  // Look for power button press and release
  if (debouncer.update())
  {
    if (debouncer.fell())
    {
      button_press_millis = current_time_millis;
#ifdef DEBUG
      Serial.println("Button press detected, starting time counter");
#endif
    }
    else if (debouncer.rose())
    {
      on_button_release();
#ifdef DEBUG
      Serial.println("Button release detected!");
#endif
    }
  }

  // Check boost converter Vout and update PWM if necessary
  update_boost_converter();

  // Check buck converter Vout and update PWM if necessary
  update_buck_converter();
}