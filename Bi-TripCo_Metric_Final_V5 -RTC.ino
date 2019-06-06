// 29 zmienna dla LPG
// 207 przerwania
// 311 menu
// 610 dystans


// Libraries
#include "PinChangeInterrupt.h" // enables the interrupts at the other pin except the ones at the digital pins 2 and 3
#include <avr/sleep.h>          // enables the sleep function
#include <avr/power.h>          // power management
#include <EEPROMex.h>           // enables some special functions for writing to and reading from EEPROM
#include <EEPROMVar.h>
#include <Wire.h>               // IIC LIBRARY
#include <math.h>               // enables complex math functions
#include <Adafruit_SSD1306.h>   // library for the OLED and graphics
#include <Adafruit_GFX.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#define ONE_WIRE_BUS A2
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
DeviceAddress outside;

#define OLED_RESET 4
Adafruit_SSD1306 display(OLED_RESET);

//        22.9    0.000346192
//        30.2    0.000456550
// variables declaration
volatile static float vss_pulse_distance = 0.000418791;  //dla 1 punktów distance coefficient -- change this according to your measurements (km/pulse)
volatile float LPG_injector_flow = 1192e-11;
volatile static float unleadedFlow = 1743e-11;            // gasoline injector flow coef. -- change this according to your measurements (litres/microseconds)

byte LPG1, LPG2, LPG3, LPG4, digitLPG, pos = 0;
volatile unsigned long vss_pulses;
volatile float traveled_distance, traveled_distance2, traveled_distance3, traveled_distance4, seconds_passed, speed, avg_speed, distance_to_LPGstation;
volatile float used_LPG, used_LPG2, instant_LPG_consumption, avg_LPG_consumption, LPG_in_tank, Full_tank = 45.00;
volatile float  srednieLPG , chwilowe, dystans, sredniePB, zasieg, zostaloLPG, poziomLPG;
volatile float used_Unleaded, used_Unleaded2;
volatile float average_L_100km_Unlead;

volatile unsigned long unleadTime1 = 0, unleadTime2 = 0, unleadinj_Open_Duration = 0;
volatile unsigned long LPG_injector_open_duration = 0, injTime1 = 0, injTime2 = 0;
int postemp;
int vss_pin = 2; // VSS signal input at digital pin 2 (INT0)
int LPG_pin = 3; // LPG injector signal input at digital pin 3 (INT1)
int ignition_pin = 9; // ignition signal input
int unleaded_pin = 12; // Unleaded injector signal input
int analogPin = A3; // poziom paliwa
float tempC;


boolean buttonState, buttonState2;
boolean lastButtonState2 = HIGH;
long lastDebounceTime = 0, logohold, counter, counter2;
volatile boolean ignition = true;
boolean ignoreRelease = false;
boolean inst_disp = true;
byte menunumber = 0,  menunumbermax = 10;
boolean timeRead = false, displaychange = true;
float thermReading, steinhart;
boolean dots = true;
//int poziom = 0;
int temperatura;


//Below are the variables declaration for the thermistor -- you can find more info at: https://learn.adafruit.com/thermistor/using-a-thermistor

#define NUMSAMPLES 5
#define BCOEFFICIENT 3630

int samples[NUMSAMPLES];
int i = 0;


void setup()
{
  /// temperatura
  Serial.begin(9600);
  Serial.println("Dallas Temperature IC Control Library Demo");

  // locate devices on the bus
  Serial.print("Locating devices...");
  sensors.begin();
  Serial.print("Found ");
  Serial.print(sensors.getDeviceCount(), DEC);
  Serial.println(" devices.");

  // report parasite power requirements
  Serial.print("Parasite power is: ");
  if (sensors.isParasitePowerMode()) Serial.println("ON");
  else Serial.println("OFF");
  if (!sensors.getAddress(outside, 0)) Serial.println("Unable to find address for Device 0");
  Serial.print("Device 0 Address: ");
  //  printAddress(outside);
  Serial.println();

  // set the resolution to 9 bit (Each Dallas/Maxim device is capable of several different resolutions)
  sensors.setResolution(outside, 9);

  Serial.print("Device 0 Resolution: ");
  Serial.print(sensors.getResolution(outside), DEC);
  Serial.println();




  pinMode(10, INPUT); // Button 2
  pinMode(11, INPUT); // Button 1

  pinMode(vss_pin, INPUT);
  pinMode(LPG_pin, INPUT);
  pinMode(ignition_pin, INPUT);
  pinMode(unleaded_pin, INPUT);

  // below recalls the values stored in case of power loss
  traveled_distance = EEPROM.readFloat(0);
  traveled_distance2 = EEPROM.readFloat (5);
  used_LPG = EEPROM.readFloat (10);
  used_LPG2 = EEPROM.readFloat(15);
  LPG_in_tank = EEPROM.readFloat(20);
  seconds_passed = EEPROM.readFloat(25);
  used_Unleaded = EEPROM.readFloat(30);
  traveled_distance3 = EEPROM.readFloat (35);
  used_Unleaded2 = EEPROM.readFloat(45);
  traveled_distance4 = EEPROM.readFloat (50);
  // LPG1 = EEPROM.readByte (40);
  // LPG2 = EEPROM.readByte (41);
  // LPG3 = EEPROM.readByte (42);
  // LPG4 = EEPROM.readByte (43);
  //LPG_injector_flow = 1043e-11;


  noInterrupts();
  // set and initialize the TIMER1
  TCCR1A = 0; // set entire TCCR1A register to 0
  TCCR1B = 0; // set entire TCCR1B register to 0
  TCCR1B |= (1 << CS12);
  TIMSK1 |= (1 << TOIE1);
  TCNT1 = 3036;

  attachPCINT(digitalPinToPinChangeInterrupt(ignition_pin), ignitionSignal, CHANGE);

  interrupts();
  delay(10);
  // Serial.begin(9600);
}

void loop()
{
 // When the ignition switch is turned, executes the next two ifs
  if (ignition == true and digitalRead(9) == LOW)
  {
    // Serial.println("pin9_LOW");
    // Serial.println(menunumber);
    detachInterrupt(digitalPinToInterrupt(vss_pin));
    detachInterrupt(digitalPinToInterrupt(LPG_pin));
    detachPCINT(digitalPinToPinChangeInterrupt(unleaded_pin));
    delay(50);
    ignition = false;





















    // when the ignition switched off it stores the values to the EEPROM
    EEPROM.writeFloat (0, traveled_distance);
    EEPROM.writeFloat (5, traveled_distance2);
    EEPROM.writeFloat (10, used_LPG);
    EEPROM.writeFloat (15, used_LPG2);
    EEPROM.writeFloat (20, LPG_in_tank);
    EEPROM.writeFloat (25, seconds_passed);
    EEPROM.writeFloat (30, used_Unleaded);
    EEPROM.writeFloat (35, traveled_distance3);
    EEPROM.writeFloat (45, used_Unleaded2);
    EEPROM.writeFloat (50, traveled_distance4);

    display.ssd1306_command(SSD1306_DISPLAYOFF);
    //  Serial.println("sleep");
    sleep_enable(); // enables the sleep mode
    set_sleep_mode(SLEEP_MODE_PWR_DOWN); // set the sleep mode
    cli();
    sleep_bod_disable(); // disables the brown out detector to consume less power while sleeping
    sei();
    sleep_mode(); // microcontroller goes to sleep
  }
  // when it wakes up continues from here -- also the first time we turn the ignition key to ON starts from here
  if (ignition == true && digitalRead(9) == HIGH)
  {
    ignition = false;    // this variable is needed in order to run once the code inside this if
    logohold = millis(); // hold the LOGO screen on, for 2 sec
    inst_disp = true;
    display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
    display.clearDisplay();
    display.setCursor(15, 10);
    display.setTextSize(2);
    display.print("FORESTER");
    display.display();

    attachInterrupt(digitalPinToInterrupt(vss_pin), distance, RISING); // attaches the interrupt which related to the VSS signal
    attachInterrupt(digitalPinToInterrupt(LPG_pin), LPG_injector_time, CHANGE); // interrupt for LPG injector signal
    attachPCINT(digitalPinToPinChangeInterrupt(unleaded_pin), UnleadedTime, CHANGE); // petrol injector signal input and interrupt
    while (millis() - logohold < 3000) ;


  }

  // every 5sec calculates average LPG consumption, remaining distance accordingly to the calculated remaining fuel in tank, average speed and average Unleaded cons.
  if (millis() % 5000 < 50)
  {
    avg_LPG_consumption = 100 * used_LPG2 / traveled_distance2;
    distance_to_LPGstation = 100 * LPG_in_tank / avg_LPG_consumption;
    avg_speed = (traveled_distance3 / seconds_passed) * 3600;
    average_L_100km_Unlead = 100 * used_Unleaded2 / traveled_distance2;
  }

  // For the short and long press function I consulted: http://jmsarduino.blogspot.gr/2009/05/click-for-press-and-hold-for-b.html
  // There are many differences though
  // "button 1" -- only  SHORT PRESS function -- changes occur on press, because there is no LONG PRESS function
  if (digitalRead(11) != buttonState) {
    buttonState = digitalRead(11);
    // Serial.println("pin11_low");
    if (buttonState == LOW)
    {
      // changes the display indications forwards
      if (menunumber <= menunumbermax) {
        if (menunumber == menunumbermax) menunumber = 0;
        else menunumber += 1;
      }
    }
  }

  ////////// "button 2" functions /////////////////
  buttonState2 = digitalRead(10);
  if (buttonState2 == LOW && lastButtonState2 == HIGH) lastDebounceTime = millis(); //keeps the time that button2 pressed

  if (buttonState2 == HIGH && lastButtonState2 == LOW)
  {
    if (ignoreRelease == false) // "button 2" SHORT PRESS functions -- on release
    {
      if (menunumber <= menunumbermax)
      {
        if (menunumber == 0) menunumber = menunumbermax;
        else menunumber = menunumber - 1;
      }
    }
    else ignoreRelease = false;
  }
  //////////////////// "button 2" LONG PRESS functions /////////////////////////////////////////////////////
  if (digitalRead(10) == HIGH) displaychange = true;
  if (buttonState2 == LOW && (millis() - lastDebounceTime) > 3000 && displaychange == true) // below are the long press functions for button 2
  {
    if (menunumber == 1) counter = millis(), counter2 = millis(), displaychange = false, digitLPG = 1; //goes to the "LPG Coef." menu
    if (menunumber == 0) traveled_distance2 = 0, used_LPG2 = 0, avg_LPG_consumption = 0, used_Unleaded2 = 0;// average LPG consumption and average Unleaeded cons. reset
    if (menunumber == 3) traveled_distance3 = 0, seconds_passed = 0, avg_speed = 0; // average speed reset
    if (menunumber == 4) traveled_distance = 0; //traveled Distance = 0
    if (menunumber == 6) used_LPG = 0; // total LPG litres = 0
    if (menunumber == 7) LPG_in_tank = Full_tank; // LPG tank reset to full
    if (menunumber == 8) average_L_100km_Unlead = 0;//goes to the "Time Setting" menu
    if (menunumber == 2) traveled_distance2 = 0, used_LPG2 = 0, avg_LPG_consumption = 0, used_Unleaded2 = 0; // same with menunumber = 1
    if (menunumber == 9) used_Unleaded = 0; // total unleaded fuel reset

    ignoreRelease = true;
  }
  lastButtonState2 = buttonState2;
  //menunumber = 0;
  (menunumber);
  chwilowe = instant_LPG_consumption;
  srednieLPG = avg_LPG_consumption;

  //Serial.println(poziom);


  // below are the misc screens
  switch (menunumber)
  {
    case 0:
cons1:
      display.clearDisplay();
      display.setTextColor(WHITE);
      display.setTextSize(1);
      display.setCursor(2, 23);
      display.print("Srednie LPG L/100KM");
      display.setCursor(40, 0);
      display.setTextSize(3);
      if (isnan(srednieLPG) || isinf(srednieLPG)) display.print(F("---"));
      else display.print(srednieLPG, 1);
      display.display();
    break;

    case 1:
        display.clearDisplay();
        if(inst_disp == true) displaychange = false, inst_disp = false, digitLPG = 0, pos = 0;
        display.setTextColor(WHITE);
        display.setCursor(19 , 23);
        display.setTextSize(1);
        display.print(F("CHWILOWE LPG"));
        display.setCursor(93,23);
        display.setTextSize(1);
        if (speed > 2) display.print(F("L/100"));
        else display.print(F("L/H")); //when the car is stopped displays the instant consumption in l/h
        display.setCursor(40,0);
        display.setTextSize(3);
        if(chwilowe > 99.9) display.print(F("--.-"));
        else display.print(chwilowe,1);
        display.display();
    break;

    case 2:
      display.clearDisplay();
      display.setTextColor(WHITE);
      display.setCursor(40, 0);
      display.setTextSize(3);
      display.print(speed, 0);
      display.setCursor(25, 24);
      display.setTextSize(1);
      display.print(F("PREDKOSC KM/H"));
      display.display();
      break;

    case 3:
      display.clearDisplay();
      display.setTextColor(WHITE);
      display.setCursor(40, 0);
      display.setTextSize(3);
      display.print(avg_speed, 0);
      display.setCursor(10, 24);
      display.setTextSize(1);
      display.print(F("SREDNIA PRED. KM/H"));;
      display.display();
      break;

    case 4:
      display.clearDisplay();
      display.setTextColor(WHITE);
      display.setCursor(19, 24);
      display.setTextSize(1);
      display.print(F("PRZEBYTE KM"));
      display.setCursor(35, 0);
      display.setTextSize(3);
      display.print(traveled_distance, 1);
      display.display();
      break;

    case 5:
      display.clearDisplay();
      display.setTextColor(WHITE);
      display.setCursor(40, 0);
      display.setTextSize(3);
      if (distance_to_LPGstation < 10 ||  isnan(distance_to_LPGstation) || isinf(distance_to_LPGstation) ) display.print(F("---"));
      else display.print(distance_to_LPGstation, 0);
      display.setTextSize(1);
      display.setCursor(25, 24);
      display.print(F("ZASIEG KM"));
      display.display();
      break;

    case 6:
      display.clearDisplay();
      display.setTextColor(WHITE);
      display.setCursor(34, 24);
      display.setTextSize(1);
      display.print(F("ZUZYTE LPG"));
      display.setCursor(40, 0);
      display.setTextSize(3);
      display.print(used_LPG, 1);
      //display.print(F("l"));
      display.display();
      break;

    case 7:
      display.clearDisplay();
      //Serial.println("case_7");
      display.setTextColor(WHITE);
      //display.drawBitmap(31, 0,  FuelTank, 64, 32, WHITE);
      display.setCursor(40, 0);
      display.setTextSize(3);
      if (LPG_in_tank <= 0 ) display.print(F("0.0"));
      else display.print(LPG_in_tank, 1);
      display.setTextSize(1);
      display.setCursor(19, 24);
      display.print(F("ZOSTALO LPG L"));
      display.display();
      break;

    case 8:
      display.clearDisplay();
      display.setTextColor(WHITE);
      display.setCursor(10, 24);
      display.setTextSize(1);
      display.print("SREDNIE PB L/100KM");
      display.setCursor(40, 0);
      display.setTextSize(3);
      if (isnan(average_L_100km_Unlead) || isinf(average_L_100km_Unlead)) display.print(F("---"));
      else display.print(average_L_100km_Unlead, 1);
      display.display();
      break;
    case 9:
      display.clearDisplay();
      display.setTextColor(WHITE);
      display.setTextSize(1);
      display.setCursor(37, 24);
      display.print(F("ZUZYTE PB"));
      display.setCursor(40, 0);
      display.setTextSize(3);
      display.print(used_Unleaded, 1);
      display.display();
      break;
    case 10:
      display.clearDisplay();
      sensors.requestTemperatures();
      float tempC = sensors.getTempCByIndex(0);
      display.setTextColor(WHITE);
      display.setCursor(40, 0);
      display.setTextSize(3);
      display.print(tempC, 1);
      display.setTextSize(1);
      display.setCursor(16, 24);
      display.print(F("TEMP. ZEWNETRZNA"));
      display.display();
      break;
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// the following are secondary displays and activated only in case we want to set the LPG Coef.(case 11) or the Time (case 12)




  }
  //Serial.println("koniec case");
}

ISR(TIMER1_OVF_vect) //TIMER1 overflow interrupt -- occurs every 1sec --
{
  instantSpeed();

  LPG_Consumption();
  LPG_injector_open_duration = 0;

  unleadedConsumption();
  unleadinj_Open_Duration = 0;

  seconds_passed++;
  vss_pulses = 0;

  TCNT1 = 3036;
}

void distance()
{
  vss_pulses++;
  traveled_distance += vss_pulse_distance;
  traveled_distance2 += vss_pulse_distance;
  traveled_distance3 += vss_pulse_distance;
  traveled_distance4 += vss_pulse_distance;
  // we calculate 3 times the same thing in order to reset the distance, the average cons. and average speed independently
}

void instantSpeed()
{
  distance_per_sec = vss_pulse_distance * vss_pulses;
  speed = (distance_per_sec * 3600);
}

// The following routine is giving the way to calculate the total time that LPG injector open during the 1sec interval
void LPG_injector_time()
{
  injTime2 = micros();
  if ((injTime2 - injTime1) < 5000 && (injTime2 - injTime1) > 4)
  {
    LPG_injector_open_duration = LPG_injector_open_duration + injTime2 - injTime1;
  }
  injTime1 = injTime2;
}


void LPG_Consumption()
{
  if (speed > 2 ) instant_LPG_consumption = (100 * ((LPG_injector_open_duration * LPG_injector_flow) * 3600)) / speed;

  else  instant_LPG_consumption = LPG_injector_open_duration * LPG_injector_flow * 3600; // when the car stops calculates the instant consumption in l/h

  used_LPG = used_LPG + (LPG_injector_open_duration * LPG_injector_flow);
  used_LPG2 = used_LPG2 + (LPG_injector_open_duration * LPG_injector_flow);
  LPG_in_tank = LPG_in_tank - (LPG_injector_open_duration * LPG_injector_flow);
}

void UnleadedTime() // it is called every time a change occurs at the gasoline injector signal and calculates gasoline injector opening time, during the 1sec interval
{
  if (digitalRead(12) == LOW)
  {
    unleadTime1 = micros();
  }
  if (digitalRead(12) == HIGH)
  {
    unleadTime2 = micros();
  }
  if (unleadTime2 > unleadTime1)
  {
    if ((unleadTime2 - unleadTime1) > 500 && (unleadTime2 - unleadTime1) < 12000) // some conditions to avoid false readings because of noise

    {
      unleadinj_Open_Duration = unleadinj_Open_Duration + (unleadTime2 - unleadTime1);
      //total useconds that the gasoline injector opens throughout 1sec
    }
  }
}

void unleadedConsumption()
{
  used_Unleaded = used_Unleaded + (unleadinj_Open_Duration * unleadedFlow);
  used_Unleaded2 = used_Unleaded2 + (unleadinj_Open_Duration * unleadedFlow);
}


void ignitionSignal() // this is called everytime the ingintion signal changes -- if the microcontroller is in sleep mode, it will wake up
{
  ignition = !ignition;
}

void printTime()
{
  DateTime dt = rtc.now();
  char sdt[15];
  sprintf(sdt, "%02d:%02d", dt.hour(), dt.minute());
  display.print(sdt);
}

void printTemperatureSensors(DeviceAddress deviceAddress)
{
  // method 1 - slower
  //Serial.print("Temp C: ");
  //Serial.print(sensors.getTempC(deviceAddress));
  //Serial.print(" Temp F: ");
  //Serial.print(sensors.getTempF(deviceAddress)); // Makes a second call to getTempC and then converts to Fahrenheit

  // method 2 - faster
  float tempC = sensors.getTempC(deviceAddress);
  if (tempC == DEVICE_DISCONNECTED_C)
  {
    Serial.println("Error: Could not read temperature data");
    return;
  }
  Serial.print("Temp C: ");
  Serial.print(tempC);
  Serial.print(" Temp F: ");
  Serial.println(DallasTemperature::toFahrenheit(tempC)); // Converts tempC to Fahrenheit
}
