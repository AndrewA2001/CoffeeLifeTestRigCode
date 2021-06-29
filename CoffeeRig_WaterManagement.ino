#include <Adafruit_MAX31865.h>
#include <Adafruit_GFX.h>
#include <Adafruit_GrayOLED.h>
#include <Adafruit_SPITFT.h>
#include <Adafruit_SPITFT_Macros.h>
#include <gfxfont.h>
#include "stdio.h"
#include "stdlib.h"
#include <Adafruit_ST7789.h> // Hardware-specific library for ST7789
#include <SPI.h>
//#include "DHT.h"

#define KILL_BTN 21
#define INLET_PUMP 9
#define OUTLET_PUMP 10
#define UNIT_FAN 11
#define VESSEL_FAN 12
#define INLET_FLOAT 34
#define OUTLET_FLOAT 36
#define DRUM_FLOAT 24
#define Pressure A14
#define Current A15
#define MAX_PUMP_TIME 20
#define MAX_FAN_TIME 20
#define NUM_BYTES 6  //4 data bytes + a start and end byte 
#define char_width 12
#define char_height 8
#define DHTPIN 32
#define DHTTYPE DHT22

//thermocouple defintions
#define RREF      4300.0
#define RNOMINAL  1000.0

////TFT Screen Pins
#define TFT_CS        48                                                    //48
#define TFT_DC        44                                                   //44
#define TFT_RST        35




int BrewStart = 0;
int vessel_fan_ovrd = 0;
int outlet_pump_ovrd = 0;
int unit_fan_ovrd = 0;
int loop_rate = 25;  //.5s
int print_rate = 50; //1s
int inlet_pump_err = 0;
int outlet_pump_err = 0;
int TC1_err = 0;
int TC2_err = 0;
int hour = 0;
int minute = 0;
int day = 0;
int initialBrewtimer = 0;
int currErr = 0;
int Unit_inactive = 0;
bool handshake = false;
bool INLET_PUMP_ON = false;
bool OUTLET_PUMP_ON = false;
bool VESSEL_FAN_ON = false;
bool UNIT_FAN_ON = false;
bool UNIT_FAN_OVRD = false;
bool Kill_Switch = false;
bool new_data = false;
char receivedBytes[6];
byte ErrorMask = 0x00;
String RigStatus = "idle";
String prevStatus = "";
char DataToPi[300];

//variables for current and pressure sensor
unsigned long previousMillis = 0;
const long interval = 1000; //200 milliseconds
float voltage, pressure, currentvolt;
int value, value2;

//DHT

//DHT dht(DHTPIN, DHTTYPE);


//status variables
int outlet_float_status = 1;
int inlet_float_status = 1;


//Timers
double inlet_Pump_timer = 0;
double outlet_Pump_timer = 0;
double unit_fan_timer = 0;
double vessel_fan_timer = 0;
unsigned int loop_timer = 0;
unsigned int loop_timer2 = 0;
unsigned int SerialPrint_timer = 0;
unsigned int Time_counter = 0;
unsigned int fan_limit = MAX_FAN_TIME;

double TC1 = 0;
double TC2 = 0;
double TC3 = 0;
double TC4 = 0;

//Functions
void PreBrew();
void CheckOverrides();
void DigitalInterrupt();
void ConnectToSystem();
void ReceiveCommands();
void ParseCommands(int numbytes);
void PrintSerial();
void UpdateErrors();
void UpdateSensorStatus();
void TFT_Init();
void LogUptime();
void TFT_print();


//Thermocouples
Adafruit_MAX31865 thermo = Adafruit_MAX31865(8, 22, 24, 26);
Adafruit_MAX31865 thermo2 = Adafruit_MAX31865(5, 22, 24, 26);
Adafruit_MAX31865 thermo3 = Adafruit_MAX31865(3, 22, 24, 26);
Adafruit_MAX31865 thermo4 = Adafruit_MAX31865(4, 22, 24, 26);

float temp1 = 0;
float temp2 = 0;
float temp3 = 0;
float temp4 = 0;

//DHT
float h = 0.0; //humidity
float t = 0.0; //Celcius Temperature

//TFT Screen
Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST); //screen tft object


void setup() {
  // put your setup code here, to run once:

  //Set up serial
  Serial.begin(9600);
  //  dht.begin();
  ADMUX = 0xC8; // turn on internal reference, right-shift ADC buffer, ADC channel = internal temp sensor




  while (!Serial) {
    /*Wait for serial port to connect*/
  }

  //Set-up I/O pins
  pinMode(KILL_BTN, INPUT_PULLUP);
  pinMode(INLET_PUMP, OUTPUT);
  pinMode(OUTLET_PUMP, OUTPUT);
  pinMode(UNIT_FAN, OUTPUT);
  pinMode(VESSEL_FAN, OUTPUT);
  pinMode(INLET_FLOAT, INPUT_PULLUP);
  pinMode(OUTLET_FLOAT, INPUT_PULLUP);
  pinMode(DRUM_FLOAT, INPUT_PULLUP);

  // Attach digital interrupt to KILL_BTN
  attachInterrupt(digitalPinToInterrupt(KILL_BTN), DigitalInterrupt, FALLING);

  //Function timer ( 16bit timer, runs at 20ms)
  TCCR1A = 0;
  OCR1A = 40000; //Timer will interrupt every 20ms
  TCCR1B = (1 << CS11) | (1 << WGM12); //Pre-scale of 8 and CTC mode
  TIMSK1 = (1 << OCIE1A); //Interrupt on compare match

  //Universal interrupts
  sei();

  //ADC BLAH
  thermo.begin(MAX31865_3WIRE);
  thermo2.begin(MAX31865_3WIRE);
  thermo3.begin(MAX31865_3WIRE);
  thermo4.begin(MAX31865_3WIRE);

  //TFT setup
  TFT_Init();

  //Pressure and Current Sensor Setup
  pinMode(Pressure, INPUT); //Pressure Sensor
  pinMode(Current, INPUT); //Current Sensor

}

void loop() {
  // put your main code here, to run repeatedly:


  if ( Kill_Switch ) {

    Serial.println("System off");
    RigStatus = "STOPPED, KILL SWITCH ACTIVE";
    digitalWrite(INLET_PUMP, LOW);
    digitalWrite(OUTLET_PUMP, LOW);
    digitalWrite(UNIT_FAN, LOW);
    digitalWrite(VESSEL_FAN, LOW);
    TFT_print("Kill Switch");


    /* Do sumn*/

  } else {

    if ( loop_timer2 >= loop_rate ) {

      UpdateSensorStatus();
      UpdateErrors();

      loop_timer2 = 0;
    }


    if ( !handshake ) { //Commands can only be received if system is connected to main controler( Pi/PC )

      ConnectToSystem();

    } else {


      if ( loop_timer >= loop_rate )  { //Functions run every .5s


        ReceiveCommands();
        //ParseCommands();
        PreBrew();
        CheckOverrides();
        loop_timer = 0;
      }

    }


    if ( SerialPrint_timer >= print_rate ) { //Function runs every 1s
      PrintSerial();
      SerialPrint_timer = 0;

    }

    if ( Time_counter >= 3000) {

      LogUptime();
      Time_counter = 0;
    }

  }


}

void TFT_Init() {

  //TFT setup
  tft.init(135, 240);
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextSize(2);
  tft.setRotation(3);
  tft.setCursor(0, 0);

  //Print header for status variables

  tft.print("Rig Status: ");
  tft.setCursor(144, 0);
  tft.print(RigStatus);
  prevStatus = RigStatus;

  //Print Time heading
  tft.setCursor(0, 33);
  tft.print("Run Time :");

}

void ConnectToSystem() {

  char receivedChar;


  if (Serial.available() > 0 & !handshake) { //Stop attempting to find "C" command if system is already connected via handshake var

    receivedChar = Serial.read();

    if (receivedChar == 'C' ) {

      //Serial.println("Arduino is connected to System");
      handshake = true;
      RigStatus = "Running";
      tft.fillRect(140, 0, 12 * (prevStatus.length()), 24, 0); //used to clear the screen
      tft.setCursor(144, 0);
      tft.print(RigStatus);

      loop_timer = 0;
    }
  } else {

    //Serial.println("No data in buffer");
  }
  delay(100);
}


void DigitalInterrupt() {

  int Kill_Button_Status = digitalRead(KILL_BTN);
  if (Kill_Button_Status == HIGH)
  {
    Kill_Switch = true; //True causes the main loop to not be executed, currently commented out due to button issues
  }
}


void ReceiveCommands() {

  char startMarker = '$';
  char EndMarker = '&';
  static byte ndx = 0;
  static bool recvInprogress = false;
  byte rb ;

  while ( Serial.available()) {

    rb = Serial.read();

    if ( recvInprogress ) {


      if ( rb == EndMarker ) {

        recvInprogress = false;
        new_data = true;
        ParseCommands(ndx);
        ndx = 0;
        //Serial.println("End Marker found");


      } else {

        receivedBytes[ndx] = rb;
        ndx++;
        //Serial.print("Receiving bytes");
      }

    } else if ( rb == startMarker ) {

      recvInprogress = true;
      //Serial.print("Start Marker found");
    }
  }
}


void PreBrew() {




  if ( BrewStart) {
    //Check status of float
    inlet_float_status = !digitalRead(INLET_FLOAT); //Returns 1 when Reservoir is full

    if (!inlet_float_status & !INLET_PUMP_ON & ( inlet_Pump_timer < MAX_PUMP_TIME)) { //Only turn on Pump if Reservois is actually low

      INLET_PUMP_ON = true;
      digitalWrite(INLET_PUMP, HIGH);

    } else if ( !inlet_float_status & INLET_PUMP_ON ) { //Reservoir still not filled , but pump running

      inlet_Pump_timer += 1;
      if ((inlet_Pump_timer >= MAX_PUMP_TIME)) { //Check if Pump has exceeded it's max run time and shut off

        INLET_PUMP_ON = false;
        digitalWrite(INLET_PUMP, LOW);
        inlet_pump_err = 1; //update flag for error reporting


      }

    } else { //Assume float is filled , stop pump

      INLET_PUMP_ON = false;
      digitalWrite(INLET_PUMP, LOW);
    }



    //Run FAN functions
    if ( !unit_fan_ovrd ) { //Do not run this section if the unit fan is currently being overridden

      if ( !UNIT_FAN_ON & (unit_fan_timer < fan_limit) ) {

        //Serial.println("I'm here");
        UNIT_FAN_ON = true;
        unit_fan_timer = 0; //Timer is shared in two different code sections , reset to ensure one does not block the other
        digitalWrite(UNIT_FAN, HIGH);

      }  else if ( UNIT_FAN_ON & (unit_fan_timer < fan_limit)) {


        unit_fan_timer += 1;
        //Serial.println(UNIT_FAN_ON);

      } else if ( UNIT_FAN_ON & (unit_fan_timer >= fan_limit)) {

        UNIT_FAN_ON = false;
        digitalWrite(UNIT_FAN, LOW);
      }
    }


    //Run outlet pump
    outlet_pump_ovrd = 1; //trigger outlet pump override, override is limited by time


    //run vessel fans
    vessel_fan_ovrd = 1; //run vessel fan override, override is limited by time


    //Get intial brew time for unit inactive error
    initialBrewtimer += 1;












  } else {  //Ensure actuators turn off when "BrewDone" command is 0

    if ( UNIT_FAN_ON ) {

      digitalWrite(UNIT_FAN, LOW);
      UNIT_FAN_ON = false;
    }

    if ( INLET_PUMP_ON ) {

      digitalWrite(INLET_PUMP, LOW);
      INLET_PUMP_ON = false;
    }
  }

}

void CheckOverrides() {

  //Outlet Pump Override



  if ( outlet_pump_ovrd) { //Override is initiated on pin

    outlet_float_status = !digitalRead(OUTLET_FLOAT); //Returns 1 when Reservoir is full

    if (!outlet_float_status & !OUTLET_PUMP_ON & ( outlet_Pump_timer < MAX_PUMP_TIME)) { //Only turn on Pump if Reservois is actually low

      OUTLET_PUMP_ON = true;
      digitalWrite(OUTLET_PUMP, HIGH);

    } else if ( !outlet_float_status & OUTLET_PUMP_ON ) { //Reservoir still not filled , but pump running

      outlet_Pump_timer += 1;
      if ((outlet_Pump_timer >= MAX_PUMP_TIME)) { //Check if Pump has exceeded it's max run time and shut off

        OUTLET_PUMP_ON = false;
        digitalWrite(OUTLET_PUMP, LOW);
        outlet_pump_err = 1; //update flag for error reporting

      }

    } else { //Assume float is filled , stop pump

      OUTLET_PUMP_ON = false;
      digitalWrite(OUTLET_PUMP, LOW);
    }
  } else if ( OUTLET_PUMP_ON & !outlet_pump_ovrd) { //Override was on, but is now off . Stop pump

    digitalWrite(OUTLET_PUMP, LOW);
    OUTLET_PUMP_ON = false;
  }

  //Vessel fan override
  if ( vessel_fan_ovrd) {

    if ( !VESSEL_FAN_ON & (vessel_fan_timer < fan_limit) ) {

      VESSEL_FAN_ON = true;
      digitalWrite(VESSEL_FAN, HIGH);

    }  else if ( VESSEL_FAN_ON & (vessel_fan_timer < fan_limit)) {


      vessel_fan_timer += 1;

    } else if ( VESSEL_FAN_ON & (vessel_fan_timer >= fan_limit)) {

      VESSEL_FAN_ON = false;
      digitalWrite(VESSEL_FAN, LOW);
    }
  } else if ( VESSEL_FAN_ON & !vessel_fan_ovrd ) {

    digitalWrite(VESSEL_FAN, LOW);
    VESSEL_FAN_ON = false;
  }

  //Unit fan override
  if (unit_fan_ovrd ) {

    if ( !UNIT_FAN_OVRD & (unit_fan_timer < MAX_FAN_TIME) ) {

      UNIT_FAN_OVRD = true;
      digitalWrite(UNIT_FAN, HIGH);
      unit_fan_timer = 0; //Timer is shared in two different code sections , reset to ensure one does not block the other

    }  else if ( UNIT_FAN_OVRD & (unit_fan_timer < MAX_FAN_TIME)) {


      unit_fan_timer += 1;

    } else if ( UNIT_FAN_OVRD & (unit_fan_timer >= MAX_FAN_TIME)) {

      UNIT_FAN_OVRD = false;
      digitalWrite(UNIT_FAN, LOW);
    }
  } else if ( UNIT_FAN_OVRD & !unit_fan_ovrd ) {

    digitalWrite(UNIT_FAN, LOW);
    UNIT_FAN_OVRD = false;
  }



}


void ParseCommands(int numbytes) {

  static char  function, command, command1, command2, command3;
  int dyn_time;
  if ( new_data ) {

    function = receivedBytes[0]; //What feature to run
    command = receivedBytes[1]; //should it be on or off
    if ( numbytes > 3 ) {

      char data[] = {receivedBytes[2], receivedBytes[3]};
      dyn_time = atoi(data) * 120;//convert to seconds
    } else {

      char data[] = {receivedBytes[2]};
      dyn_time = atoi(data) * 120; //convert to seconds
      Serial.println(data);
      Serial.println(atoi(data));
      Serial.println(dyn_time);


    }


    new_data = false;


    if ( function == 'A') {

      if ( command == '1' ) {

        BrewStart = 1;
        //Init and Reset timers
        inlet_Pump_timer = 0;
        unit_fan_timer = 0;
        ( dyn_time == 0 ) ? ( fan_limit = MAX_FAN_TIME ) : ( fan_limit = dyn_time ); //if parsing time failed ( dyn_time == 0 ) then assing the limit to be pre-defined MAX

      } else {

        BrewStart = 0;

      }
    }
    //vessel fan B
    if (function == 'B')
    {

      if ( command == '1') {

        vessel_fan_ovrd = 1;
        command = '0';
        //Reset timer
        vessel_fan_timer = 0;
        ( dyn_time == 0 ) ? ( fan_limit = MAX_FAN_TIME ) : ( fan_limit = dyn_time ); //if parsing time failed ( dyn_time == 0 ) then assing the limit to be pre-defined MAX
        //Serial.print("VFO Status :");
        //Serial.println(vessel_fan_ovrd);
      } else {

        vessel_fan_ovrd = 0;
      }
    }
    //outlet pump  C

    if (function == 'C')
    {
      if ( command == '1') {

        outlet_pump_ovrd = 1;
        //Reset Timer
        outlet_Pump_timer = 0;

        //Print to TFT board
        //Serial.print("OPO status :");
        //Serial.println(outlet_pump_ovrd);
      } else {

        outlet_pump_ovrd = 0;
      }
    }

    if (function == 'D')
    {

      if ( command == '1') {

        unit_fan_ovrd = 1;
        unit_fan_timer = 0;
        command3 = '0';
        //Print to TFT board
        //Serial.print("UFO status :");
        //Serial.println(unit_fan_ovrd);
        ( dyn_time == 0 ) ? ( fan_limit = MAX_FAN_TIME ) : ( fan_limit = dyn_time ); //if parsing time failed ( dyn_time == 0 ) then assing the limit to be pre-defined MAX


      } else {

        unit_fan_ovrd = 0;
      }
    }
  }
}



void PrintSerial() {

  //Section to print serial data
  // Protocol :




  // Serial.println(ErrorMask,HEX);
  //Serial.write("\n");
  //How do we clear Error masks ?
  //Commmunicate information to Pi
  //Protocol : Error Mask | TC1 | TC2 | TC3 | TC4 | Current | Pressure |  Vessel fan status| inlet pump status| outlet pump status| inlet float | outlet float | Unit fan status | inlet pump timer | outlet pump timer | unit fan timer |
  //sprintf(DataToPi,"%X %s %s",ErrorMask,TC1,TC2);
  Serial.print(ErrorMask, HEX);
  Serial.print(' ');
  Serial.print(TC1);
  Serial.print(' ');
  Serial.print(TC2);
  Serial.print(' ');
  Serial.print(TC3);
  Serial.print(' ');
  Serial.print(TC4);
  Serial.print(' ');
  Serial.print(String(currentvolt) + " A");
  Serial.print(' ');
  Serial.print(String(pressure) + " kpi");
  Serial.print(' ');
  Serial.print(VESSEL_FAN_ON); //Print vessel fan status
  Serial.print(' ');
  Serial.print(INLET_PUMP_ON); //Print inlet pump status
  Serial.print(' ');
  Serial.print(OUTLET_PUMP_ON); // outlet pump status
  Serial.print(' ');
  Serial.print(inlet_float_status); //inle float status
  Serial.print(' ');
  Serial.print(outlet_float_status); //inle float status
  Serial.print(' ');
  if ( unit_fan_ovrd ) {

    Serial.print(UNIT_FAN_OVRD);
    Serial.print(' ');
  } else {

    Serial.print(UNIT_FAN_ON);
    Serial.print(' ');
  }
  Serial.print(vessel_fan_timer);
  Serial.print(' ');
  Serial.print(inlet_Pump_timer);
  Serial.print(' ');
  Serial.print(outlet_Pump_timer);
  Serial.print(' ');
  Serial.print(unit_fan_timer);
  Serial.println(' '); //End of serial stream





}

void UpdateErrors() {


  //Check for TC errors
  uint8_t fault = thermo.readFault();
  uint8_t fault2 = thermo2.readFault();
  if (fault) {

    TC1_err = 1;
    if ((fault & MAX31865_FAULT_HIGHTHRESH) || (fault & MAX31865_FAULT_LOWTHRESH)) {
      TC1_err = 1;
    }
    thermo.clearFault();

  } else {

    TC1_err = 0;
  }

  if (fault2) {

    TC2_err = 1;
    if ((fault2 & MAX31865_FAULT_HIGHTHRESH) || (fault2 & MAX31865_FAULT_LOWTHRESH)) {
      TC2_err = 1;
    }
    thermo2.clearFault();

  } else {

    TC2_err = 0;
  }


  //Error 3 ( reservoir float active before prew )
  if ( outlet_float_status ) {

    outlet_pump_err = 1; //vessel is full( this should never happen) , set an error
  }


  //Err4 Unit inactive
  if ( initialBrewtimer >= 15 && !currErr) {

    currErr = 1;
    if ( currentvolt <= 2.5 ) {

      Unit_inactive = 1;
    }
  }


  //Combine all Errors into one mask with protocol : INlet pump| outlet pump| TC 1-3 OOR Errors |

  ErrorMask = (inlet_pump_err << 7) | (outlet_pump_err << 6) | (TC1_err << 5) | (TC2_err << 4) | (Unit_inactive << 3);

  TFT_print("Error Status");





}

void UpdateSensorStatus() {

  //Update Status of float sensor
  inlet_float_status = !digitalRead(INLET_FLOAT);
  outlet_float_status = !digitalRead(OUTLET_FLOAT);

  //Collect TC temperature
  TC1 = thermo.temperature(RNOMINAL, RREF);
  TC2 = thermo2.temperature(RNOMINAL, RREF);
  TC3 = thermo3.temperature(RNOMINAL, RREF);
  TC4 = thermo4.temperature(RNOMINAL, RREF);

  //Pressure Sensor/Current Sensor
  //  value = analogRead(A14); //Analog read pressure sensor
  value2 = analogRead(A15); //analog read current sensor

  voltage = value * (5.0 / 1023.0); //Convert from analog to digital
  currentvolt = value2 * (5.0 / 1023.0) * 10;

  //convert voltage to pressure?
  pressure = voltage - 0.50;
  pressure = pressure * 50;
  pressure = pressure / 4;
  pressure = pressure * 6.89476;
  //Serial.println(String(value) + ',' + String(voltage) + ',' + String(pressure) + ',' + String(currentvolt));



  //Add DHT

  //  h = dht.readHumidity();
  // Read temperature as Celsius (the default)
  //  t = dht.readTemperature();

}


void LogUptime() {



  minute += 1;

  if ( minute > 59 ) { // if you get to 59 minutes, increment hour and reset minute

    hour += 1;
    minute = 0;
    if ( hour > 23 ) { //if it is 23:59, reset to a day and 00:00
      day += 1;
      hour = 0;
    }
  }



  //  tft.fillRect(0, 80, 20 * 12, 32, 0); //x,y ( in pixels ) width and height of rect, color in hex .
  //  tft.setCursor(0, 80);
  //  tft.print(day);
  //  tft.print(" days,");
  //  tft.print(hour);
  //  tft.print(" hrs,");
  //  tft.print(minute);
  //  tft.print(" mins");
  TFT_print("Run Time");




}

void TFT_print(char* feature) {

  if ( !(strcmp(feature, "Rig Status"))) { //If Rig status needs to be updated

    tft.fillRect(140, 0, char_width * (prevStatus.length()), char_height, 0); //used to clear the screen
    tft.setCursor(144, 0);
    tft.print(RigStatus);
  } else if (!(strcmp(feature, "Run Time"))) {

    tft.fillRect(0, 66, 20 * char_width, 2 * char_height, 0); //x,y ( in pixels ) width and height of rect, color in hex .
    tft.setCursor(0, 49);
    tft.print(day);
    tft.print(" days,");
    tft.print(hour);
    tft.print(" hrs,");
    tft.print(minute);
    tft.print(" mins");
  } else if ( !(strcmp(feature, "Error Status"))) {

    tft.fillRect(0, 98, 20 * char_width, 2 * char_height, 0); //x,y ( in pixels ) width and height of rect, color in hex .
    tft.setCursor(0, 99);
    tft.print("Error Code:");
    tft.print(ErrorMask, 16); //Print Error status in Hex

  }
  else if (!(strcmp(feature, "Kill Switch"))) {


    tft.fillScreen(ST77XX_BLACK);
    tft.setTextSize(3);
    tft.setCursor(135 / 2, 0);
    tft.print("STOP, KILL SWITCH ACTIVE");
  } else {

    //Do nothing
  }


}





ISR(TIMER1_COMPA_vect) {

  loop_timer += 1;
  loop_timer2 += 1;
  SerialPrint_timer += 1;
  //counter for time functinoality
  if ( handshake ) {//Only run this section when the system is able to receive commands ( Status is "running")

    Time_counter += 1;
  }
}
