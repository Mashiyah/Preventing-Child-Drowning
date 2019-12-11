// Version 0.9 - Debugging

// Include Libraries :
//------------------------------------------------------------------------------------------------------------------------------------//
#include <SoftwareSerial.h>       // Allow serial communication on other digital pins of the Arduino(built-in librarie) - for SIM800L
#include <Wire.h>                 // Communicate with I2C (built-in librarie) - for MS5803-14BA pressure sensor

#include <LowPower.h>             // Power management library (https://github.com/rocketscream/Low-Power)
#include <SparkFun_MS5803_I2C.h>  // Allows easily talk to the MS5803-14BA pressure sensor (http://librarymanager/All#SparkFun_MS5803-14BA)
#include <Battery.h>              // Allows monitor battery consumption (https://github.com/rlogiacco/BatterySense)

SoftwareSerial SIM800L_Serial(11, 10);  // TX and RX pins (respectively) to communicate with SIM800L module
MS5803 pressure_sensor(ADDRESS_HIGH);   // Available addresses (selected by jumper on board) - ADDRESS_HIGH = 0x76 (default) OR ADDRESS_LOW  = 0x77
Battery batt = Battery(3000, 4200, A0); // minVoltage, maxVoltage and sensePin
//------------------------------------------------------------------------------------------------------------------------------------//

// Define Pins :
//------------------------------------------------------------------------------------------------------------------------------------//
#define InWater_Prob 2   // The pin number for the Prob (Only pin 2,3 USABLE FOR INTERRUPTS)
#define Button 3         // The pin number for the Push button (Only pin 2,3 USABLE FOR INTERRUPTS)
#define RedledPin 8      // The pin number for the Red LED
#define GreenledPin 9    // The pin number for the Green LED

// Pinout for TB6612FNG
#define PWMA 6      // PWM input that controls the speed
#define AIN1 5      // One of the two inputs that determines the direction.
#define STBY 7      // Allows the H-bridges to work when high
#define AIN2 4      // One of the two inputs that determines the direction.
//------------------------------------------------------------------------------------------------------------------------------------//

// Global Variables :
//------------------------------------------------------------------------------------------------------------------------------------//
volatile boolean InWater = false;       // Flag for symbolizes the contact with water (volatile because associated with interrupt)
volatile boolean Check_dev = false;     // Flag that indicates test status or not (volatile because associated with interrupt)
boolean Arduino_state = false;          // Flag for the Arduino status (stand-by = true, sleep = false)
boolean Valve_state = false;            // Flag for the valve status (open = true, close = false)
boolean Gsm_sleepMode = false;          // Flag for the SIM800L status (stand-by = false, sleep = true)
//------------------------------------------------------------------------------------------------------------------------------------//

// Function Declaration :
//------------------------------------------------------------------------------------------------------------------------------------//
void Water_activated_switch();
void Button_activated_switch();
void In_water();
double Check_depth(double pressure_baseline);
void Drowning_detected();
void Toggle_valve();
void Sending_sms(String message);
void Check_device();
//------------------------------------------------------------------------------------------------------------------------------------//

/* --- setup() ------------------------------------------------------------------------------------------------------------------
   This function is called when a sketch starts. We use it to initialize variables, pin modes, start using libraries, etc.
   The function will only run once, after each powerup or reset of the Arduino board.
   Input: None
   Output: None
*/
void setup() {

  pinMode(InWater_Prob, INPUT);     // Initialize the Prob pin as an input for water contact********************לבדוק אם לשים INPUT_PULLUP משפראת המצב
  pinMode(Button, INPUT_PULLUP);    // Initialize the button pin as an input with pull up resistor

  // PinMode for common Anode RG
  pinMode(RedledPin, OUTPUT);       // initialize the Red LED pin as an output
  pinMode(GreenledPin, OUTPUT);     // initialize the Green LED pin as an output

  digitalWrite(RedledPin, 255);     // set initial LED state
  digitalWrite(GreenledPin, 255);   // set initial LED state

  // PinMode for TB6612FNG
  pinMode(STBY, OUTPUT);            // PWM input that controls the speed
  pinMode(AIN1, OUTPUT);            // One of the two inputs that determines the direction
  pinMode(AIN2, OUTPUT);            // Allows the H-bridges to work when high
  pinMode(PWMA, OUTPUT);            // One of the two inputs that determines the direction

  analogReference(INTERNAL);        // Configures the reference voltage used for analog input volts

  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);

  Serial.println(F("====================================== START SETUP ====================================================="));


  // Start I2C object for the MS5803-14BA pressure sensor
  Wire.begin();

  Serial.print("Initialize the MS5803-14BA pressure sensor - ");
  pressure_sensor.reset();                  // Reset device I2C
  if (!pressure_sensor.begin()) {            // Initialize library for subsequent pressure measurements
    Serial.println("Done successfully");
  }
  else {
    Serial.println("Unsuccessful");
  }

  Serial.print("Initialize the GSM SIM800L module - ");
  SIM800L_Serial.begin(9600);                // Initialize Serial Monitor for the SIM800L
  delay(50);

  SIM800L_Serial.println("AT");
  if ((readSerial().indexOf("OK")) != -1 ) {
    Serial.println("Done successfully");
  }
  else {
    Serial.println("Unsuccessful");
  }

  Serial.println("Initialize Monitor battery consumption - Done successfully\n");
  batt.begin(1100, 5.9651, &sigmoidal);     // Initialize monitor battery consumption

  Check_dev = true;             // Indicates that test mode enable.
  Check_device();
  Check_dev = false;            // After finish the Check_device, the flag returns to its initial value


  Serial.print("\nGSM SIM800L module entering Sleep mode - ");
  SIM800L_Serial.println("AT+CSCLK=2");     // Entering Sleep mode
  if ((readSerial().indexOf("OK")) != -1 ) {
    Serial.println("Successfully");
    Gsm_sleepMode = true;                   // Indicates that the SIM800L is in sleep mode
  }
  else {
    Serial.println("Unsuccessful");
  }

  attachInterrupt(digitalPinToInterrupt(2), Water_activated_switch, RISING);    // When pin 2 rising change, exit sleep mode and execute Water_activated_switch function
  attachInterrupt(digitalPinToInterrupt(3), Button_activated_switch, FALLING);  // At the push of a button, exit sleep mode and execute Button_activated_switch function

  Serial.println(F("Putting the Arduino into power down state - Sleep forever until interrupt"));
  Serial.println(F("====================================== END SETUP =======================================================\n"));
  delay (10000);
  LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF); // Putting microcontroller into power down state - Sleep forever until interrupt
}
//------------------------------------------------------------------------------------------------------------------------------------//

/* --- loop() --------------------------------------------------------------------------------------------------------------------
   The function does precisely what its name suggests, and loops consecutively.
   Enable / disable interrupt, enable sleep mode, and enable drowning detection mechanisms or Check device mechanisms.
   Input: None
   Output: None
*/
void loop() {

  if (InWater) {
    detachInterrupt(digitalPinToInterrupt(2));  // Remove interrupt to stop it from keep firing
    detachInterrupt(digitalPinToInterrupt(3));  // Remove interrupt to stop it from keep firing

    In_water();                   // Function is activated after exiting sleep mode when water contact is detected
    InWater = false;              // After leaving the water, the flag returns to its initial value
    Arduino_state = true;         // Indicates that the Arduino is in stand-by mode
    attachInterrupt(digitalPinToInterrupt(2), Water_activated_switch, RISING);    // When pin 2 rising change, exit sleep mode and execute Water_activated_switch function
    delay(100);
  }

  if (Check_dev) {
    detachInterrupt(digitalPinToInterrupt(3));    // Remove interrupt to stop it from keep firing

    Check_device();               // Function is activated after exiting sleep mode when push of a button is detected
    Check_dev = false;            // After finish the Check_device, the flag returns to its initial value
    Arduino_state = true;         // Indicates that the Arduino is in stand-by mode
    attachInterrupt(digitalPinToInterrupt(3), Button_activated_switch, FALLING);  // At the push of a button, exit sleep mode and execute Button_activated_switch function
    delay(100);
  }

  if (Arduino_state) {
    Serial.println(F("\nPutting the Arduino into power down state - Sleep forever until interrupt"));
    Arduino_state = false; // After finish the Check_device or In_water, the flag returns to its initial value (sleep)
    delay (100);
    LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);  // Putting microcontroller into power down state - Sleep forever until interrupt
  }
}
//------------------------------------------------------------------------------------------------------------------------------------//

/* --- Water_activated_switch --------------------------------------------------------------------------------------------------
   This function is called when pin 2 rising change (interrupt function).
   Switches the flag to water contact mode.
   Input: None
   Output: None
*/
void Water_activated_switch() {
  InWater = true;                             // Indicates that water contact detected.
  Check_dev = false;                          // After finish the Check_device, the flag returns to its initial value
}
//------------------------------------------------------------------------------------------------------------------------------------//

/* --- Button_activated_switch() --------------------------------------------------------------------------------------------------
   This function is called when pin 3 falling change (push of a button) (interrupt function).
   Switches the flag to Check device mode.

*/
void Button_activated_switch() {
  Check_dev = true;                             // Indicates that push of a button detected.
}
//------------------------------------------------------------------------------------------------------------------------------------//

/* --- In_water() ---------------------------------------------------------------------------------------------------------------
   This function is called by loop() after the interrupt function ended.
   Initial pressure gauge, after a certain amount of time, measures again and then sends the data
   to the Check_depth() function to calculate the altitude difference.
   If the height difference is below the threshold (for a predefined time) then the Drowning_detected() function is activated.
   Otherwise, the function ends and returns to the loop()function.
   Input: None
   Output: None
*/
void In_water() {

  const float depth_threshold = -70.0;  // The threshold depth from which a drowning is defined (Negative as we go down deep)

  // Create variables to store results from the pressure sensor
  double altitude_delta, pressure_baseline;

  int timer_count = 0; // Used to measure time below the threshold

  Serial.println("Contact with water detected - SYSTEM ON\n");
  Serial.println("Start measuring pressure:\n");

  pressure_baseline = pressure_sensor.getPressure(ADC_4096); // Read initial pressure measurement in mbar for reference (Po)

  while (digitalRead(2) == HIGH)  {   // As long as there is contact with water, calculate altitude delta
    altitude_delta = Check_depth(pressure_baseline);
    while (altitude_delta <= depth_threshold && digitalRead(2) == HIGH)  {  // As long as below the depth threshold, turn on timer
      if (timer_count >= 2000) {         // 8 * 250 msec = 2 sec = 2000 msec
        Drowning_detected();             // Identified a drowning case
        break;
      }
      Serial.print("Time below the threshold (msec)= ");
      delay(250);
      timer_count = timer_count + 250;
      Serial.println(timer_count);
      altitude_delta = Check_depth(pressure_baseline);
    }
    timer_count = 0;
    delay(500);
  }
  Serial.println("Contact with water NOT detected - SYSTEM OFF");
}
//------------------------------------------------------------------------------------------------------------------------------------//

/* --- Check_depth() ----------------------------------------------------------------------------------------------------------
   This function is called by In_water() when we need to calculate the altitude delta.
   Initial pressure gauge, after a certain amount of time, measures again and calculate the altitude difference.
   Input: pressure_baseline (Po) for reference.
   Output: altitude_delta.
*/
double Check_depth(double pressure_baseline) {

  // Create variables to store results from the pressure sensor
  double pressure_abs, pressure_avg = 0.0, altitude_delta = 0.0;

  // Perform a number of pressure measurements
  // and average calculation for more accurate pressure
  for (int i = 0; i < 10; i++)
  {
    // Read pressure from the sensor in mbar.
    pressure_abs = pressure_sensor.getPressure(ADC_4096);
    delay(100); // 100 msec delay
    pressure_avg = pressure_avg + pressure_abs;
  }
  pressure_avg = pressure_avg / 10.0;

  Serial.print("Pressure baseline (mbar)= ");
  Serial.println(pressure_baseline);
  Serial.print("Pressure avg (mbar)= ");
  Serial.println(pressure_avg);


  /* Taking our baseline pressure at the beginning we can find an approximate
    change in altitude based on the differences in pressure.
    Given a pressure measurement P (mbar) and the pressure at a baseline P0 (mbar),
    return altitude (millimeters) above baseline.*/

  altitude_delta = (44330.0 * (1 - pow(pressure_avg / pressure_baseline, 1 / 5.255)));
  Serial.print("Altitude change (mm) = ");
  Serial.println(altitude_delta);
  Serial.println("");
  return altitude_delta;
}
//------------------------------------------------------------------------------------------------------------------------------------//

/* --- Drowning_detected() --------------------------------------------------------------------------------------------------
   This function is called by In_water() when we have identified a drowning case.
   The function scheduling two different drowning prevention functions.
   The first function of opening the valve is to release the Co2.
   Second function for sending an emergency message to a control center.
   Input: None
   Output: None
*/
void Drowning_detected() {

  Serial.println(F("Drowning_detected!!"));
  Serial.println(F("==========================Activated drowning prevention functions=========================="));
  Toggle_valve();                               // Toggle the valve using the DC motor
  Sending_sms(String("Drowning detected!!"));   // Sending an emergency message to a control center with the SIM800L
  Toggle_valve();                               // Toggle the valve using the DC motor
  Serial.println(F("==========================Finish drowning prevention functions==========================\n"));
  Serial.println(F("Continue to measuring pressure:\n"));

}
//------------------------------------------------------------------------------------------------------------------------------------//

/* --- Toggle_valve() ---------------------------------------------------------------------------------------------------------
   This function is called by Drowning_detected() when we have identified a drowning case.
   If the flag is false (the valve is closed) start the engine controller and turn the engine to open the valve and vice versa
   Input: None
   Output: None
*/
void Toggle_valve() {
  if (!Valve_state) { // If the valve close turn valve on
    Serial.println("\nTurn valve ON\n");

    // Turn valve on
    digitalWrite(STBY, HIGH);
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    analogWrite(PWMA, 255);
    delay(5000);

    Valve_state = true;     // Indicates that valve in open

    // Stop the motor
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW);
    analogWrite(PWMA, 0);
    delay(2000);
  }
  else {
    Serial.println("\nTurn valve OFF\n");

    // Turn valve off
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    analogWrite(PWMA, 255);
    delay(5000);

    Valve_state = false;    // Indicates that valve in close

    // Stop the motor
    digitalWrite(STBY, LOW);
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW);
    analogWrite(PWMA, 0);
  }
}
//------------------------------------------------------------------------------------------------------------------------------------//

/* --- Sending_sms() ----------------------------------------------------------------------------------------------------------
   This function is called by Drowning_detected() when we have identified a drowning case
   or by Check_device() when we have identified push of a button.
   Wakes up the SIM800L and sends SMS (depending on the state we in) to a control center.
   After sending messages, go back to sleep.
   Input: String message
   Output: None
*/
void Sending_sms(String message) {

  const String number = "547280910";    // Control Center Emergency Number

  SIM800L_Serial.println("AT");         // A dummy message to wake the component
  delay(150);
  Gsm_sleepMode = false;                // Indicates that the SIM800L is in stand-by mode

  if (Valve_state) {
    Serial.println("Sending emergency SMS to the control center number");

    for (int i = 0; i < 3; i++) {
      SIM800L_Serial.println("AT+CMGF=1");                       // Set the Mode as Text Mode
      delay(150);
      SIM800L_Serial.println("AT+CMGS=\"+972" + number + "\"");  // To send a message to a control center
      delay(150);
      SIM800L_Serial.print(message);                             // Content of the SMS message
      delay(150);
      SIM800L_Serial.write((byte)0x1A);                          // End of message character 0x1A : Equivalent to Ctrl+z
      delay(50);
      SIM800L_Serial.println();
      delay(3000);
    }
  }
  else if (Check_dev) {
    Serial.println("Sending report SMS to the control center");

    SIM800L_Serial.println("AT+CMGF=1");                       // Set the Mode as Text Mode
    delay(150);
    SIM800L_Serial.println("AT+CMGS=\"+972" + number + "\"");  // To send a message to a control center
    delay(150);
    SIM800L_Serial.print(message);                             // Content of the SMS message
    delay(150);
    SIM800L_Serial.write((byte)0x1A);                          // End of message character 0x1A : Equivalent to Ctrl+z
    delay(50);
    SIM800L_Serial.println();
    delay(3000);
  }

  if (Gsm_sleepMode) {
    Serial.print("GSM SIM800L module entering Sleep mode - ");
    SIM800L_Serial.println("AT+CSCLK=2");     // Entering Sleep mode
    if ((readSerial().indexOf("OK")) != -1 ) {
      Serial.println("Successfully");
      Gsm_sleepMode = true;                   // Indicates that the SIM800L is in sleep mode
    }
    else {
      Serial.println("Unsuccessful");
    }
  }
}
//------------------------------------------------------------------------------------------------------------------------------------//

/* --- Check_device() ----------------------------------------------------------------------------------------------------------
   This function is called by loop() when we have identified a push buttan case.
   Checks all system components and then sends an appropriate SMS to a control center.
   If all components are correct, a green LED turns on, otherwise red.
   Input: None
   Output: None
*/
void Check_device() {
  Serial.println("Testing process begins:");

  boolean fault = false;              // Flag that indicates if the device is working properly (true = Not good)
  boolean MS5803_state = false;       // Flag that indicates if the MS5803-14BA pressure sensor is working properly (true = OK, false = NOT ok)
  String message = "";

  int batt_volt = batt.voltage();
  int batt_percent = batt.level();
  float temperature_c = pressure_sensor.getTemperature(CELSIUS, ADC_256);   // Temperature measurement to check the sensor pressure

  if (temperature_c >= -20 && temperature_c <= 40) {
    Serial.println("The MS5803-14BA pressure sensor - OK");
    MS5803_state = true;
  }
  else {
    Serial.println("The MS5803-14BA pressure sensor - NOT OK");
    MS5803_state = false;
    fault = true;
  }

  SIM800L_Serial.println("AT"); // A dummy message to wake the component
  delay(200);

  SIM800L_Serial.println("AT");
  if ((readSerial().indexOf("OK")) != -1 ) {
    Serial.println("The SIM800L GSM module - OK");
  }
  else {
    Serial.println("The SIM800L module - NOT OK");
    fault = true;
  }

  Serial.print("Battery voltage is ");
  Serial.print(batt_volt);
  Serial.print("mV (");
  Serial.print(batt_percent);
  Serial.println("%)");

  if (batt_volt >= 3300) {
    Serial.println("The percentage of battery is within the allowable range");
  }
  else {
    Serial.println("The percentage of battery is NOT within the allowable range");
    fault = true;
  }

  if (fault) {                        // One of the components is not working or the battery is low
    message = message + "The device NOT working properly, Battery Level: " + String(batt_percent) + "%";
    digitalWrite(RedledPin, 0);       // Low battery -> Red led ON
    digitalWrite(GreenledPin, 255);   // Low battery -> Green led OFF
  }
  else {                              //All components work and the battery is good
    message = message + "The device is working properly, Battery Level: " + String(batt_percent) + "%";
    digitalWrite(RedledPin, 255);     // OK battery -> Red Led OFF
    digitalWrite(GreenledPin, 0);     // OK battery -> Green led ON
  }

  delay(5000);

  Sending_sms(message);                   // Sending report message to a control center with the SIM800L

  digitalWrite(RedledPin, 255);           // Turns off the Red LED
  digitalWrite(GreenledPin, 255);         // Turns off the Green LED
}
//------------------------------------------------------------------------------------------------------------------------------------//

/* --- readSerial() ----------------------------------------------------------------------------------------------------------
   The function returns the output of the SIM800L.
   Input: None
   Output: SIM800L_Serial.readString - The output of the SIM800L.
*/
String readSerial() {
  int  _timeout = 0;
  while  (!SIM800L_Serial.available() && _timeout < 1000  )   {
    delay(13);
    _timeout++;
  }
  if (SIM800L_Serial.available()) {
    return SIM800L_Serial.readString();
  }
}
