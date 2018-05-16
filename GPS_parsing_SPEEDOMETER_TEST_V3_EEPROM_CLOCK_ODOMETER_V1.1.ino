//GPS Speedometer and clock for Hudson Stepdown Gauge Cluster
//Tach will need to be run from seperate Arduino (as far a my programming ability will allow)

//v1.1 5/13/2018

// Test code for Adafruit GPS modules using MTK3329/MTK3339 driver
//
// This code shows how to listen to the GPS module in an interrupt
// which allows the program to have more 'freedom' - just parse
// when a new NMEA sentence is available! Then access data when
// desired.
//
// Tested and works great with the Adafruit Ultimate GPS module
// using MTK33x9 chipset
//    ------> http://www.adafruit.com/products/746
// Pick one up today at the Adafruit electronics shop
// and help support open source hardware & software! -ada

#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
#include <SwitecX25.h>
//#include <SwitecX25c.h>
#include <EEPROMex.h>
#include <microsmooth.h>

#define STEPS 944
#define CLOCK_STEPS 8640

#define HOUR_OFFSET -6L //-7 MST, -6 MDT

SwitecX25 motor1(STEPS, 4, 5, 6, 7);
SwitecX25 motor2(CLOCK_STEPS, A0, A1, A2, A3);
SwitecX25 motor3(STEPS, 10, 11, 12, 13);

unsigned long previousMillis1 = 0;
unsigned long previousMillis2 = 0;
long OdometerOut = EEPROM.readLong(355);
float OdometerTot = EEPROM.readLong(355) * 1.0;
float previousElev = 0;
float deltaElev = 0;
float deltaEspeed = 0;
float actualSpeed = 0;
//long tachPos = 0;

uint16_t *history = ms_init(EMA);

const int maxAllowedWrites = 100000;
const int memBase = 355;

int addressLong;
int addressFloat;

const unsigned char PS_16 = (1 << ADPS2);
const unsigned char PS_32 = (1 << ADPS2) | (1 << ADPS0);
const unsigned char PS_64 = (1 << ADPS2) | (1 << ADPS1);
const unsigned char PS_128 = (1 << ADPS2) | (1 << ADPS1) | (ADPS1 << ADPS0);

// If you're using a GPS module:
// Connect the GPS Power pin to 5V
// Connect the GPS Ground pin to ground
// If using software serial (sketch example default):
//   Connect the GPS TX (transmit) pin to Digital 3
//   Connect the GPS RX (receive) pin to Digital 2
// If using hardware serial (e.g. Arduino Mega):
//   Connect the GPS TX (transmit) pin to Arduino RX1, RX2 or RX3
//   Connect the GPS RX (receive) pin to matching TX1, TX2 or TX3

// If you're using the Adafruit GPS shield, change
// SoftwareSerial mySerial(3, 2); -> SoftwareSerial mySerial(8, 7);
// and make sure the switch is set to SoftSerial

// If using software serial, keep this line enabled
// (you can change the pin numbers to match your wiring):
SoftwareSerial mySerial(3, 2);

// If using hardware serial (e.g. Arduino Mega), comment out the
// above SoftwareSerial line, and enable this line instead
// (you can change the Serial number to match your wiring):

//HardwareSerial mySerial = Serial1;

Adafruit_GPS GPS(&mySerial);

// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences.
#define GPSECHO  false

// this keeps track of whether we're using the interrupt
// off by default!
boolean usingInterrupt = false;
void useInterrupt(boolean); // Func prototype keeps Arduino 0023 happy

void setup()
{
  ADCSRA &= ~PS_128; // remove bits set by Arduino library
  ADCSRA |= PS_128; // set our own prescaler to 16, fastest
  motor1.zero();
  //motor2.zero();
  motor1.setPosition(STEPS);
  motor1.updateBlocking();
  delay(750);
  motor1.setPosition(0);
  motor1.updateBlocking();

  //motor3.zero();
  //motor3.setPosition(STEPS);
  //motor3.updateBlocking();

  //Homeing for clock - BK-gnd, WT-pwr(270ohm<-5v), GY-sig(10kohm<-5v)
  motor2.currentStep = 8639;
  while (digitalRead(9) == HIGH) {
    //motor2.zero();
    delay(1);
    motor2.stepDown();
  }
  motor2.currentStep = 0;

  pinMode(A0, INPUT);

  // connect at 115200 so we can read the GPS fast enough and echo without dropping chars
  // also spit it out
  Serial.begin(115200);
  Serial.println("Speedomter using Adafruit GPS library!");

  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(9600);

  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // uncomment this line to turn on only the "minimum recommended" data
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since
  // the parser doesn't care about other sentences at this time

  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
  // For the parsing code to work nicely and have time to sort thru the data, and
  // print it out we don't suggest using anything higher than 1 Hz

  // Request updates on antenna status, comment out to keep quiet
  GPS.sendCommand(PGCMD_ANTENNA);

  // the nice thing about this code is you can have a timer0 interrupt go off
  // every 1 millisecond, and read data from the GPS for you. that makes the
  // loop code a heck of a lot easier!
  useInterrupt(false);

  delay(500);
  // Ask for firmware version
  mySerial.println(PMTK_Q_RELEASE);

  EEPROM.setMemPool(memBase, EEPROMSizeUno);
  EEPROM.setMaxAllowedWrites(maxAllowedWrites);
  delay(100);

  addressLong = EEPROM.getAddress(sizeof(long));
  addressFloat = EEPROM.getAddress(sizeof(float));
  Serial.println(addressLong);
  Serial.println(addressFloat);

  OdometerRead();

}

// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
#ifdef UDR0
  if (GPSECHO)
    if (c) UDR0 = c;
  // writing direct to UDR0 is much much faster than Serial.print
  // but only one character can be written at a time.
#endif
}

void useInterrupt(boolean v) {
  if (v) {
    // Timer0 is already used for millis() - we'll just interrupt somewhere
    // in the middle and call the "Compare A" function above
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
  } else {
    // do not call the interrupt function COMPA anymore
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
  }
}

void loop()                     // run over and over again
{
  byte interval2 = 25;
  int interval1 = 2000;
  float gaugePos;
  unsigned long currentMillis1 = millis();
  unsigned long currentMillis2 = millis();
  unsigned int speedRange = 104;  //104MPH @ 944 steps for Hudson Inner Speedometer reading

  int tachRead = analogRead(A0);
  //int tachProcessed = ema_filter(tachRead, history);

  // in case you are not using the interrupt above, you'll
  // need to 'hand query' the GPS, not suggested :(
  if (! usingInterrupt) {
    // read data from the GPS in the 'main loop'
    char c = GPS.read();
    // if you want to debug, this is a good time to do it!
    if (GPSECHO)
      if (c) Serial.print(c);
  }

  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences!
    // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
    //Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false

    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
  }



  // approximately every 2 seconds or so, print out the current stats
  currentMillis1 = millis();
  if ((unsigned long)(currentMillis1 - previousMillis1) >=  interval1) {

    Serial.print("\nTime: ");
    Serial.print(GPS.hour, DEC); Serial.print(':'); //MDT
    Serial.print(GPS.minute, DEC); Serial.print(':');
    Serial.print(GPS.seconds, DEC); Serial.print('.');
    Serial.println(GPS.milliseconds);
    Serial.print("Date: ");
    Serial.print(GPS.day, DEC); Serial.print('/');
    Serial.print(GPS.month, DEC); Serial.print("/20");
    Serial.println(GPS.year, DEC);//*/
    Serial.print("Fix: "); Serial.print((int)GPS.fix);
    Serial.print(" quality: "); Serial.println((int)GPS.fixquality); //*/

    if (GPS.fix) {
      /*Serial.print("Location: ");
        Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
        Serial.print(", ");
        Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);
        Serial.print("Location (in degrees, works with Google Maps): ");
        Serial.print(GPS.latitudeDegrees, 4);
        Serial.print(", ");
        Serial.println(GPS.longitudeDegrees, 4);
      */

      ClockUpdate();
      Serial.print("Actual Interval (ms): "); Serial.println((currentMillis2 - previousMillis2));
      Serial.print("Speed (mph): "); Serial.println(GPS.speed * 1.150779);
      Serial.print("delta Elevation (mi): "); Serial.println(deltaElev);
      Serial.print("Elevation Speed (mph): "); Serial.println(deltaEspeed);
      Serial.print("Actual Speed (mph): "); Serial.println(actualSpeed);
      Serial.print("Speed difference (mph): "); Serial.println(actualSpeed - GPS.speed * 1.150779);
      Serial.print("Angle: "); Serial.println(GPS.angle);
      Serial.print("Altitude (ft): "); Serial.println(GPS.altitude * 3.28083);
      Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
      Serial.println(OdometerTot);
      Serial.println(OdometerOut);
      //Serial.println(timeval);
      //Serial.println(tachRead);
      Serial.println();
    }
    else {
      Serial.println("NO FIX");
      gaugePos = 90;
      motor1.setPosition(gaugePos);
      motor1.updateBlocking();
      gaugePos = 0;
      motor1.setPosition(gaugePos);
      motor1.updateBlocking();
      //motor3.update();
    }

    //gaugePos = map(gaugeProcessed, 0, 880, 0, STEPS);
    //int tachProcessed = ema_filter(tachRead, history);
    //tachPos = map(tachRead, 0, 780, 0, STEPS);  //Six Cylinder
    //motor3.setPosition(tachPos);

    previousMillis1 = currentMillis1;
  }

  if ((unsigned long)(currentMillis2 - previousMillis2) >= interval2 && GPS.fix) {
    //Serial.print("Actual Interval (ms): "); Serial.println(actual_interval);

    //gaugePos = map(gaugeProcessed, 0, 880, 0, STEPS);
    //int tachProcessed = ema_filter(tachRead, history);
    //tachPos = map(tachRead, 0, 780, 0, STEPS);  //Six Cylinder
    //motor3.setPosition(tachPos);

    if (GPS.speed < 0.29) {
      gaugePos = 0;
      actualSpeed = 0;
      motor1.setPosition(gaugePos);  //Comment out for Position Test
      //Position Test
      //int PosTest = map(100, 0, (speedRange * 10), 0, STEPS);
      //Serial.println(PosTest);
      //motor1.setPosition(PosTest);
    }
    else {
      float currentElev = (GPS.altitude * 0.00062); //current altitude in miles
      deltaElev = abs(previousElev - currentElev);  //change in elevation since last loop
      if (deltaElev > 0.1 * currentElev ) {
        deltaEspeed = 0;
      }
      else {
        deltaEspeed = deltaElev / ((currentMillis2 - previousMillis2) / 1000.0 / 3600.0);  //speed of elevation change in mph
        if (deltaEspeed > GPS.speed * 0.15) {
          deltaEspeed = GPS.speed * 0.15;
        }
      }
      //deltaEspeed = 0;
      previousElev = currentElev;
      if (GPS.speed < 0.29) {
        deltaEspeed = 0;
      }

      actualSpeed = sqrt(pow(GPS.speed * 1.15, 2) + pow(deltaEspeed, 2));  //actual speed vector
      //actualSpeed = 25.0;

      //if (actualSpeed > 0.25 && actualSpeed < 90.0) {  //Try to acount for "SPOOFING" or misreads at start-up
      OdometerTot = OdometerTot + actualSpeed * ((currentMillis2 - previousMillis2) / 1000.0 / 3600.0); //Odometer totaler, since program start
      //}

      if (EEPROM.isReady()) {
        EEPROM.updateLong(addressLong, long(OdometerTot));
      }
      else {
        Serial.println("EEPROM not ready");
      }
      if (EEPROM.isReady()) {
        OdometerOut = EEPROM.readLong(addressLong);
      }
      else {
        Serial.println("EEPROM not ready");
      }
      //OdometerTot = OdometerTot + 60.0 * ( actual_interval / 1000.0 ) / 3600.0;
      gaugePos = map((actualSpeed * 10), 0, (speedRange * 10), 0, STEPS);
      //gaugePos = map((GPS.angle * 10.0), 0, 3600, 0, STEPS);
      //gaugePos = map((GPS.altitude * 3.28), 5000, 6000, 0, STEPS);
      //previousElev = currentElev;  //set previous elevation for next loop iteration
      motor1.setPosition(gaugePos);
    }
    previousMillis2 = currentMillis2;

  }
  /*else {
    //Serial.println("NO FIX");
    gaugePos = 90;
    motor1.setPosition(gaugePos);
    motor1.updateBlocking();
    gaugePos = 0;
    motor1.setPosition(gaugePos);
    motor1.updateBlocking();
    previousMillis2 = currentMillis2;
  }*/

  //ClockUpdate();
  //motor3.updateBlocking();

  motor1.updateBlocking();


}

void ClockUpdate() {
  // Grab the current hours, minutes, seconds from the GPS.
  int hours = GPS.hour + HOUR_OFFSET;  // Add hour offset to convert from UTC
  // to local time.
  // Handle when UTC + offset wraps around to a negative or > 23 value.
  if (hours < 0) {
    hours = 24 + hours;
  }
  if (hours > 23) {
    hours = 24 - hours;
  }

  int minutes = GPS.minute;
  int seconds = GPS.seconds;

  Serial.println(hours);
  Serial.println(minutes);
  Serial.println(seconds);

  float timeval = (((hours) * 3600.0) + ((minutes) * 60.0) + (seconds));
  //Serial.println(timeval);

  timeval = map(timeval, 0, 86400, 0, CLOCK_STEPS);

  //Serial.println(timeval);

  if (timeval == 0) {
    motor2.currentStep = 0;
  }
  motor2.setPosition(timeval);

  //noInterrupts();
  //motor1.updateBlocking();
  motor2.update();

  //interrupts();

}

void OdometerRead() {
  long temp_num = 0;
  long index = 0;
  int gaugePos = 0;
  unsigned int speedRange = 104;  //105MPH @ 944 steps for Hudson Inner Speedometer reading

  //Test physical gauge number positions
  //long TestPosition = 1928374650;

  //String temp_string = String(TestPosition);
  String temp_string = String(OdometerOut);
  String char_to_int;

  temp_num = OdometerOut;
  //temp_num = TestPosition;

  //temp_num = OdometerOut;
  //temp_num = TestPosition;
  Serial.println(temp_string.length());
  Serial.println();
  do
  {
    char thisChar = temp_string[index];
    ++index;
    char_to_int = (char)thisChar;
    Serial.println(thisChar);  //Output individual numbers
    gaugePos = map((char_to_int.toInt() * 100), 0, (speedRange * 10), 0, STEPS);
    motor1.setPosition(gaugePos);
    motor1.updateBlocking();
    delay(1000);
    Serial.println();
  }
  while (index != temp_string.length());

  motor1.setPosition(STEPS);  //Show Odometer output is done with Wag of needle
  motor1.updateBlocking();
  delay(750);
  motor1.setPosition(0);
  motor1.updateBlocking();
}

