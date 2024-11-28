#include <Wire.h>
#include <Arduino.h> // Ensure the Arduino core library is included
#include "INA219.h"
#include <LiquidCrystal_I2C.h>
#include <VL53L0X.h>
#include "HX711.h"

/// LCD display
INA219 INA(0x40);
LiquidCrystal_I2C lcd(0x27, 20, 4);
unsigned long lastLcdUpdate = 0; // Last LCD update time
const int lcdUpdateIntervalMs = 1000; // Set update interval for LCD (in ms)

//anemometer  ; CONE: inside , CUP OUTSIDE
const int windspeeed_input_o = A6 ;  /// CUP anemometer
int windspeeed_input_i = 2; /// CONE anemoemter
float windSpeedOffset = 0; // Stores the calibration offset
unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
unsigned long debounceDelay = 1000;    // the debounce time; increase if the output flickers 
int wind_Count_i=0;
float wind_pin_reading;
float wind_speed_i;  
float wind_speed_o; 
float calibrated_wind_speed_i;  
float calibrated_wind_speed_o; 

/// RPM
VL53L0X RPM_sensor;
volatile int RPM_pulseCount = 0;  
float rpm = 0;
int n_strips = 1;

/// Power sensor INA219
float bus_V ;
float shunt_mV ;
float current_mA;
float power_mW ;

// FORCE TRANSDUCER 
HX711 loadcell;
const int forcetrans_input = A6;   
float forcetransOffset = 0; // Stores the calibration offset
float calibrated_forcetrans_reading;

//set VARIABLES
const float samplingFrequencyms = 1000.0 / 50; //type how many hz 
unsigned long lastSampleTime = 0; // Last sample time for all the sensors
unsigned long currentTime;   /// time for all sensors 


/////////////////////// set functions ///////////////////////////////////////
void RPM_countPulse() 
{
  RPM_pulseCount++;
}

void wind_onChange()
{
   if ( digitalRead(windspeeed_input_i) == LOW )
      wind_Count_i++;
}

void wind_read()
{
  if ((millis() - lastDebounceTime) > debounceDelay) {
    /// take CONE reading
    lastDebounceTime = millis();
    wind_speed_i = wind_Count_i*0.0875 ;
    calibrated_wind_speed_i = wind_speed_i *0.887 + 0.949 ; /// calibrate wind speed
    wind_Count_i=0;

    wind_pin_reading = analogRead(windspeeed_input_o);
    wind_pin_reading -= windSpeedOffset;
    wind_speed_o = (wind_pin_reading * 5.0/1023 - 0.4 )/1.6*32.4;  
    calibrated_wind_speed_o = wind_speed_o*1.33 + 0.969 ; 

  }
}

void RPM_read() {

  static unsigned long RPM_lastCalcTime = 0;
  static unsigned int pulseCount = 0;
  unsigned long RPM_currentTime = millis();

  unsigned long RPM_elapsedTime = RPM_currentTime - RPM_lastCalcTime;

  // Count pulses (example logic for distance sensor)
  float distance = RPM_sensor.readRangeContinuousMillimeters();
  float distance_from_sensor = 50; // Distance from sensor to blade tip

  if (distance < distance_from_sensor) { // Trigger when the blade passes
    pulseCount++;
  }

  // Calculate and print RPM every 1 second
  if (RPM_elapsedTime >= 1000) {
    float rpm = (pulseCount * 60.0) / (RPM_elapsedTime*n_strips / 1000.0); // Pulses per second * 60

    pulseCount = 0; // Reset pulse count
    RPM_lastCalcTime = RPM_currentTime;
  }
}

void power_read()
{ 
  // Read INA219 values
  bus_V = INA.getBusVoltage();
  shunt_mV = INA.getShuntVoltage_mV();
  current_mA = INA.getCurrent_mA();
  power_mW = INA.getPower_mW();
}

void force_read()
{
  float forcetrans_reading = analogRead(forcetrans_input);
  forcetrans_reading -= forcetransOffset;
  calibrated_forcetrans_reading = map(forcetrans_reading, 0, 1023, 5, 500);
}

void run_calibrate()
{
  float initialReadings = 0;
  int calibrationSamples = 20;

  for (int i = 0; i < calibrationSamples; i++) {
      initialReadings += analogRead(windspeeed_input_o);
      delay(50); // Short delay between readings
  }

  // Calculate average offset
  windSpeedOffset = initialReadings / calibrationSamples;
  Serial.print("windSpeedOffset : ");
  Serial.println(windSpeedOffset);
}

/// for LCD reading outputs
void run_updateLCD() {
  lcd.clear();               // Clear LCD display screen

  // You may need to adjust the cursor positions based on your LCD's size
  lcd.setCursor(0, 0);      // Row 0
  lcd.print("W_o: ");
  lcd.print(calibrated_wind_speed_o, 2); // Wind speed
  
  lcd.setCursor(0, 1);      // Row 0
  lcd.print("W_i: ");
  lcd.print(calibrated_wind_speed_i, 2); // Wind speed

  lcd.setCursor(0, 2);      // Row 1
  lcd.print("R: ");
  lcd.print(rpm,1); // RPM (or calculate based on your logic)

  lcd.setCursor(0, 3);      // Row 2
  lcd.print("V: ");
  lcd.print(bus_V, 2); // Bus Voltage
  
  lcd.setCursor(9, 2);      // Row 3
  lcd.print("A: ");
  lcd.print(current_mA, 2); // Current in mA

  lcd.setCursor(9, 2);      // Row 3
  lcd.print("F: ");
  lcd.print(calibrated_forcetrans_reading, 2); // Current in mA
}

/// 
void run_serial_Sensors() {

  // Serial output for python
  Serial.print(currentTime);
  Serial.print(',');
  Serial.print(calibrated_wind_speed_i);
  Serial.print(',');
  Serial.print(calibrated_wind_speed_o);
  Serial.print(',');
  Serial.print(rpm);
  Serial.print(',');
  Serial.print(bus_V);
  Serial.print(',');
  Serial.print(current_mA); 
  Serial.print(',');
  Serial.print(power_mW);
  Serial.print(',');
  Serial.println(calibrated_forcetrans_reading);
}

//////////////////////////////////////////////////// setup  ////////////////////////////////////////


void setup() {
  Serial.begin(9600);
  analogReference(DEFAULT); 

  /// wind sensor setup
  pinMode(windspeeed_input_o, INPUT);   /// CUP
  pinMode( windspeeed_input_i, INPUT_PULLUP);  
  attachInterrupt( digitalPinToInterrupt(windspeeed_input_i), wind_onChange, FALLING);

  //INA219 setup
  Wire.begin();
  if (!INA.begin() )
  { Serial.println("Could not connect INA219."); }

  INA.setMaxCurrentShunt(2, 0.1);
  delay(1000);
  Serial.println(INA.getBusVoltageRange());

  /// RPM
  // if(!RPM_sensor.init())
  // { 
  //   while (1) {}
  // }
  // RPM_sensor.startContinuous(100);

  /// LCD setup
  lcd.init();  
  lcd.backlight();  
  lcd.setCursor(1, 0);         
  lcd.print("Calibrating ...... "); 

  // delay(3000);  
  // run_calibrate();

  lcd.setCursor(1, 0);         
  lcd.print("All ready, START"); 

}

void loop() {

  currentTime = millis();

  wind_read();
  power_read();
  RPM_read();
  force_read();

  if (currentTime - lastSampleTime >= samplingFrequencyms) {
    // Read and process sensor data
    run_serial_Sensors();
    lastSampleTime = currentTime; // Update last sample time
  }

  if (currentTime - lastLcdUpdate >= lcdUpdateIntervalMs) { // Update every second
    run_updateLCD();
    lastLcdUpdate = currentTime; // Update last LCD update time
  }
  delay(500);
}
