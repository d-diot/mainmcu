/*
 * This is the sketch for the ATMega328P MCU of the d-diot board v.3.0
 */

#include <Arduino.h>

// ########################### CONFIG ##########################################

// General definition
#define ENV_PLATFORMIO    //Uncomment to make the source code compatible with Platformio and not with Arduino-mk.
#define ENABLE_STATUS_LED //Uncomment to enable the status LED (On when the pi is OFF and viceversa)
//#define EXTERNAL_BOOST    //Uncomment to disable the on-board boost converter and use an external ones
//#define EXTERNAL_BUCK     //Uncomment to disable the on-board boost converter and use an external ones
#define DEBUG //Uncomment to enable the debug messages in the serial output
//#define BB_DEBUG          //Uncomment to enable the buck and boost debug messages in the serial output
//#define DEBUG_DELAY 1500  //Uncomment to slow down the code speed in debug mode
//#define ENABLE_SLEEP      //EXPERIMENTAL: uncomment to enable MCU sleep when Pi is OFF. Works only with external buck and boost converters

// Boost Converter parameters
int BoostTargetVoltage = 10000; //Desired output voltage of the boost converter (Millivolts)
long BoostR1 = 10000;           //R1 value in Ohm (feedback circuit)
long BoostR2 = 3300;            //R2 value in Ohm (feedback circuit)
int BoostPwm = 0;               //Initial value of boost PWM (0 = off)
int BoostMaxVoltage = 20000;    //Maximum voltage allowed by the circuit design (Millivolts)
int BoostMaxPwm = 200;          //Maximum PWM value (safety features)

// Buck converter parameters
int BuckTargetVoltage = 1700; //Desired output voltage of the buck converter (Millivolts)
long BuckR1 = 0;              //R1 value in Ohm (feedback circuit, 0 = no feedback circuit)
long BuckR2 = 0;              //R2 value in Ohm (feedback circuit, 0 = no feedback circuit)
int BuckPwm = 255;            //Initial value of buck PWM (255 = off)
int BuckMinVoltage = 1500;    //Minimum voltage allowed by the circuit design

// Button parameters
int DebounceInterval = 5;         //Debounce interval in milliseconds
uint16_t ClickMinTime = 25;       //Min press time that defines a short click
uint16_t ClickMaxTime = 500;      //Max press time that defines a short click
uint16_t LongPressMinTime = 3000; //Min press time that defines a long press

// Pi Shutdown pin parameters
unsigned long pi_shutdown_pin_press_time = 300; // Time in milliseconds to keep the pi shutdown pin LOW

// Pi power state monitoring time
uint16_t pi_poweroff_bounce_time = 5000; // Time of active monitoring in milliseconds to wait before confirming the status of the on / off state of the Pi

// Power line opening delay time
unsigned long power_lines_wait_time = 500; // Time to wait after opening a power line. Necessary to avoid peak of current adsorption

// ####################### END OF CONFIG ###########################################

// ########################### INIT ################################################

// PIN definition
int ButtonPin = 2;          //Button PIN: is the same for physical button and TTP223 capacitive touch switch with jumper A closed and B open (see https://www.hackster.io/najad/how-to-use-a-ttp223-based-touch-switch-a04f7d).
int BoostPin = 3;           //Digital pin D3 for boost PWM signal
int PiPowerOffPin = 4;      //Pin connected to the Raspberry Pi poweroff signal PIN (GPIO6)
int PiShutdownPin = 7;      //Pin connected to the Raspberry Pi shutdown PIN (GPIO5)
int PiPowerlinePin = 8;     //Pin that controls the main power line (Raspberry Pi)
int BuckPin = 11;           //Digital pin D11 for buck PWM signal
int BoostPowerlinePin = 12; //Pin that controls the buck and oost converter power line
int StatusLedPin = 13;      //Pin that control the status LED
int VPowerlinePin = A0;     //Pin that controls the +5V and 3.3V (AMS117) power line
int OnPowerPin = A1;        //Pin that controls the on / off state of the system when the board is powered for the first time
int BoostFeedbackPin = A6;  //The boost feedback input is A6 (pin 20)
int BuckFeedbackPin = A7;   //The buck feedback input is A7 (pin 21)

long Vcc;
bool pi_poweroff_pin_status;
bool pi_shutdown_pin_activated = false;
bool monitor_pi_status = false;
uint8_t pi_status; // 0=OFF, 1=ON, 2=Standby, 3=power line OFF and Pi poweroff pin ON
bool PiPowerlineStatus;
bool BoostPowerlineStatus;
bool VPowerlineStatus;
unsigned long last_pi_poweroff_pin_change_time = 0;
unsigned long current_time_millis;
unsigned long button_press_millis = 0;
unsigned long button_release_millis = 0;
unsigned long pi_shutdown_pin_press_start = 0;
long BoostAcutalVoltage;
long BuckAcutalVoltage;
#ifdef ENABLE_SLEEP
bool go_to_sleep = false;
bool wake_up = true;
#endif
#ifdef ENV_PLATFORMIO
#include <Bounce2.h>
#ifdef ENABLE_SLEEP
#include <Sleep_n0m1.h>
#endif
#endif
#ifndef ENV_PLATFORMIO
#include "libraries/Bounce2/Bounce2.cpp"
#include "libraries/Bounce2/Bounce2.h"
#ifdef ENABLE_SLEEP
#include "libraries/Sleep_n0m1/Sleep_n0m1.cpp"
#include "libraries/Sleep_n0m1/Sleep_n0m1.h"
#endif
#endif
Bounce debouncer = Bounce();
#ifdef ENABLE_SLEEP
Sleep sleep;
#endif

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

void check_power_lines_status()
{
  PiPowerlineStatus = !digitalRead(PiPowerlinePin);
  BoostPowerlineStatus = !digitalRead(BoostPowerlinePin);
  VPowerlineStatus = !digitalRead(VPowerlineStatus);
#ifdef DEBUG
  Serial.print("Pi power line status: ");
  Serial.println(PiPowerlineStatus);
  Serial.print("Boost power line status: ");
  Serial.println(BoostPowerlineStatus);
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
    pi_status = 1;
#ifdef DEBUG
    Serial.println("Pi is ON: power line and poweroff pin in agreement");
#endif
  }
  else if (!PiPowerlineStatus && !current_poweroff_pin_status)
  {
    pi_status = 0;
#ifdef DEBUG
    Serial.println("Pi is OFF: power line and poweroff pin in agreement");
#endif
  }
  else if (PiPowerlineStatus && !current_poweroff_pin_status)
  {
    pi_status = 2;
#ifdef DEBUG
    Serial.println("Pi is ON but in standby: power line is ON, but the Pi is signaling a power OFF state, probably a shutdown is finished");
#endif
  }
  else if (!PiPowerlineStatus && current_poweroff_pin_status)
  {
    pi_status = 3;
#ifdef DEBUG
    Serial.println("Pi is OFF: power line is OFF, but the Pi is signaling a false power ON state due to the pulldown resistor R7");
#endif
  }
}

void update_boost_converter()
{
#ifndef EXTERNAL_BOOST
  //If the desired value is HIGHER than the real value, we increase PWM width
#ifdef BB_DEBUG
  Serial.println("Boost converter");
#endif
  if (BoostTargetVoltage > BoostMaxVoltage || !BoostPowerlineStatus)
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
#ifdef BB_DEBUG
  Serial.println("Buck converter");
#endif
  if (BuckTargetVoltage < BuckMinVoltage || !BoostPowerlineStatus)
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
  if (pi_shutdown_pin_activated)
  {
    digitalWrite(PiShutdownPin, LOW);
    pi_shutdown_pin_press_start = 0;
    pi_shutdown_pin_activated = false;
#ifdef DEBUG
    Serial.println("Pi shutdown pin released");
#endif
  }
}

void open_pi_powerline()
{
  digitalWrite(PiPowerlinePin, LOW);
  PiPowerlineStatus = true;
  delay(power_lines_wait_time);
#ifdef DEBUG
  Serial.println("Pi power line opened");
#endif
}

void open_boost_power_line()
{
  digitalWrite(BoostPowerlinePin, LOW);
  BoostPowerlineStatus = true;
  delay(power_lines_wait_time);
#ifdef DEBUG
  Serial.println("Boost power line opened");
#endif
}

void open_v_power_line()
{
  digitalWrite(VPowerlinePin, LOW);
  VPowerlineStatus = true;
  delay(power_lines_wait_time);
#ifdef DEBUG
  Serial.println("V power line opened");
#endif
}

void open_all_powerlines()
{
  open_pi_powerline();
  open_boost_power_line();
  open_v_power_line();
}

void close_all_powerlines()
{
  digitalWrite(PiPowerlinePin, HIGH);
  digitalWrite(BoostPowerlinePin, HIGH);
  digitalWrite(VPowerlinePin, HIGH);
  PiPowerlineStatus = false;
  BoostPowerlineStatus = false;
  VPowerlineStatus = false;
#ifdef DEBUG
  Serial.println("All power lines closed");
#endif
}

void on_button_release()
{
  button_release_millis = current_time_millis;
  update_pi_status();
  // Short press detection (50 - 500 ms)
  if (current_time_millis - button_press_millis >= ClickMinTime && current_time_millis - button_press_millis <= ClickMaxTime)
  {
#ifdef DEBUG
    Serial.println("Button click type: short");
#endif
    if (pi_status == 1)
    {
      activate_pi_shutdown_pin();
#ifdef DEBUG
      Serial.println("Action: Pi is ON, shutdown triggered");
#endif
    }
    else if (pi_status == 0 || pi_status == 3)
    {
      open_all_powerlines();
      monitor_pi_status = true;
#ifdef DEBUG
      Serial.println("Action: Pi is OFF, opening all power lines");
#endif
    }
    else if (pi_status == 2)
    {
      monitor_pi_status = true;
    }
  }
  // Long press detection (> 3000 ms)
  else if (current_time_millis - button_press_millis >= LongPressMinTime)
  {
#ifdef DEBUG
    Serial.println("Button click type: long");
#endif
    if (PiPowerlineStatus)
    {
      close_all_powerlines();
#ifdef DEBUG
      Serial.println("Action: PWR lines are ON, closing them all");
#endif
    }
    else
    {
      open_all_powerlines();
#ifdef DEBUG
      Serial.println("Action: PWR lines are OFF, opening them all");
#endif
    }
  }
  // Unknown click time
  else
  {
    button_release_millis = 0;
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
  pinMode(BoostPin, OUTPUT);
  pinMode(BuckPin, OUTPUT);
  pinMode(PiShutdownPin, OUTPUT);
  pinMode(StatusLedPin, OUTPUT);
  TCCR2B = TCCR2B & B11111000 | B00000001; // pin 3 and 11 PWM frequency of 31372.55 Hz
  analogWrite(BoostPin, BoostPwm);
  analogWrite(BuckPin, BuckPwm);
  // Check the power line status on startup, the switch the PIN to output mode
  pinMode(PiPowerlinePin, INPUT);
  pinMode(BoostPowerlinePin, INPUT);
  pinMode(VPowerlinePin, INPUT);
  PiPowerlineStatus = !digitalRead(PiPowerlinePin);
  BoostPowerlineStatus = !digitalRead(BoostPowerlinePin);
  VPowerlineStatus = !digitalRead(VPowerlinePin);
  pinMode(PiPowerlinePin, OUTPUT);
  pinMode(BoostPowerlinePin, OUTPUT);
  pinMode(VPowerlinePin, OUTPUT);
  PiPowerlineStatus ? digitalWrite(PiPowerlinePin, LOW) : digitalWrite(PiPowerlinePin, HIGH);
  BoostPowerlineStatus ? digitalWrite(BoostPowerlinePin, LOW) : digitalWrite(BoostPowerlinePin, HIGH);
  VPowerlinePin ? digitalWrite(VPowerlinePin, LOW) : digitalWrite(VPowerlinePin, HIGH);
  // Check the OnPowerPin status and power on or off the hub accordingly
  digitalRead(OnPowerPin) ? close_all_powerlines() : open_all_powerlines();
  // Read the Pi poweroff pin status
  pi_poweroff_pin_status = digitalRead(PiPowerOffPin);
  last_pi_poweroff_pin_change_time = millis();
  // Deactivate the Pi shutdown PIN
  digitalWrite(PiShutdownPin, LOW);
  // Monitor pi status
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

  // Post wake up actions
#ifdef ENABLE_SLEEP
  if (wake_up)
  {
    wake_up = false;
    detachInterrupt(digitalPinToInterrupt(ButtonPin));
#ifdef DEBUG
    Serial.println("Wake up: Interrupt detached");
#endif
  }
#endif

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
    Serial.print("Pi poweroff pin status change detected: status=");
    Serial.println(pi_poweroff_pin_status);
    Serial.println("Start monitoring");
#endif
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
#ifdef DEBUG
      Serial.println("Button release detected!");
#endif
      on_button_release();
    }
  }

  // Monitoring the pi status
  if (monitor_pi_status)
  {
    update_pi_status();
    if (current_time_millis - last_pi_poweroff_pin_change_time >= pi_poweroff_bounce_time && current_time_millis - button_release_millis >= pi_poweroff_bounce_time)
    {
      monitor_pi_status = false;
#ifdef DEBUG
      Serial.println("Monitoring finished: take action");
#endif
      if (pi_status == 1)
      {
        open_all_powerlines();
#ifdef ENABLE_STATUS_LED
        digitalWrite(StatusLedPin, LOW);
#endif
#ifdef DEBUG
        Serial.println("Action taken: all power lines opened");
#endif
      }
      else if (pi_status == 0 || pi_status == 2 || pi_status == 3)
      {
        close_all_powerlines();
#ifdef ENABLE_STATUS_LED
        digitalWrite(StatusLedPin, HIGH);
#endif
#ifdef DEBUG
        Serial.println("Action taken: all power lines closed");
#endif
#ifdef ENABLE_SLEEP
        go_to_sleep = true;
#ifdef DEBUG
        Serial.println("Action taken: sleep triggered");
#endif
#endif
      }
    }
  }

  // Check boost and boost converter Vout and update PWM if necessary
  update_boost_converter();
  update_buck_converter();

#ifdef DEBUG
#ifdef DEBUG_DELAY
  delay(DEBUG_DELAY);
#endif
#endif

#ifdef ENABLE_SLEEP
  if (go_to_sleep)
  {
    go_to_sleep = false;
    wake_up = true;
#ifndef EXTERNAL_BUCK
    analogWrite(BuckPin, 0);
    digitalWrite(BuckPin, HIGH);
    digitalRead(BuckFeedbackPin);
#endif
#ifndef EXTERNAL_BOOST
    analogWrite(BoostPin, 0);
    digitalWrite(BoostPin, LOW);
    digitalRead(BoostFeedbackPin);
#endif
#ifdef DEBUG
    Serial.println("Sleeping");
#endif
    delay(500);
    sleep.pwrDownMode();                     //set sleep mode
    sleep.sleepPinInterrupt(ButtonPin, LOW); //(interrupt Pin Number, interrupt State)
  }
#endif
}