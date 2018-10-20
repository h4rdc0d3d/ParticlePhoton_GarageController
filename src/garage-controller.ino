/*******************************************************************************
 * Project  : GarageController
 * Compiler : Particle Photon
 * Verion   : 0.8.0-rc.10
 * Date     : 15.10.2018
 * Author   : Tim Hornikel
 * License  : GNU General Public License v3+
*******************************************************************************/



/* HC-SR04 Ping / Range finder wiring:
 * -----------------------------------
 * Particle - HC-SR04
 *      GND - GND
 *      VIN - VCC
 *       D4 - TRIG
 *       D5 - ECHO
 */

// includings of 3rd party libraries
// @todo: check necessary includings
#include "application.h"
#include "PietteTech_DHT.h" // Temperature, Humidity Sensor Libray
#include "math.h"

 // system defines
 // @todo: check necessary defines
#define DHTTYPE  DHT22              // Sensor type DHT11/21/22/AM2301/AM2302
#define DHTPIN   D1           	    // Digital pin for communications
#define DHT_SAMPLE_INTERVAL   10000  // Sample every ten seconds

/*
 * NOTE: Use of callback_wrapper has been deprecated but left in this example
 *       to confirm backwards compabibility.  Look at DHT_2sensor for how
 *       to write code without the callback_wrapper
 */
// declaration
void dht_wrapper(); // must be declared before the lib initialization

// Lib instantiate
// ref: https://www.sparkfun.com/datasheets/Sensors/Temperature/DHT22.pdf
PietteTech_DHT DHT(DHTPIN, DHTTYPE, dht_wrapper);

// Select external Antenna
STARTUP(WiFi.selectAntenna(ANT_EXTERNAL));

// globals
unsigned int DHTnextSampleTime;	    // Next time we want to start sample
bool bDHTstarted;		    // flag to indicate we started acquisition
int n;                              // counter
String sensorStatus = "Initialization";
// Version MAJOR.MINOR.PATCH
String firmwareVersion = "0.1.0";
double dHumidity = 0.0;
double dTemperature = 0.0;
int pushButton = D0;


// Setup
void setup() {

  // serial communications
  Serial.begin(9600);
  // Wait for a USB serial connection for up to 5 seconds
  waitFor(Serial.isConnected, 5000);
  Serial.printlnf("System version: %s", System.version().c_str());
  Serial.printlnf("Firmware version: %s", firmwareVersion.c_str());
  Serial.println("---------------");

  // particle variables
  Particle.variable("Status", sensorStatus);
  Particle.variable("Temperature", &dTemperature, DOUBLE);
  Particle.variable("Humidity", &dHumidity, DOUBLE);

  // Definition of I/O-Pins
  pinMode(pushButton, OUTPUT);

  DHTnextSampleTime = 0;  // Start the first sample immediately

}


/*
 * NOTE:  Use of callback_wrapper has been deprecated but left in this example
 * to confirm backwards compatibility.
 */
 // This wrapper is in charge of calling
 // must be defined like this for the lib work
void dht_wrapper() {

  DHT.isrCallback();

}

/*
 *
 */
void readTempHumid() {

  // Check if we need to start the next sample
  if (millis() > DHTnextSampleTime) {
    if (!bDHTstarted) {		// start the sample
      Serial.print("\n\n");
      Serial.print(n);
      Serial.print(": Retrieving information from sensor: ");
      DHT.acquire();
      bDHTstarted = true;
    }

  if (!DHT.acquiring()) {		// has sample completed?

    // get DHT status
    int result = DHT.getStatus();

    Serial.print("Read sensor: ");

    switch (result) {

      case DHTLIB_OK:
        Serial.println("OK");
        sensorStatus = "OK";
        break;

      case DHTLIB_ERROR_CHECKSUM:
        Serial.println("Error\n\r\tChecksum error");
        sensorStatus = "Error\n\r\tChecksum error";
        break;

      case DHTLIB_ERROR_ISR_TIMEOUT:
        Serial.println("Error\n\r\tISR time out error");
        sensorStatus = "Error\n\r\tISR time out error";
        break;

      case DHTLIB_ERROR_RESPONSE_TIMEOUT:
        Serial.println("Error\n\r\tResponse time out error");
        sensorStatus = "Error\n\r\tResponse time out error";
        break;

      case DHTLIB_ERROR_DATA_TIMEOUT:
        Serial.println("Error\n\r\tData time out error");
        sensorStatus = "Error\n\r\tData time out error";
        break;

      case DHTLIB_ERROR_ACQUIRING:
        Serial.println("Error\n\r\tAcquiring");
        sensorStatus = "Error\n\r\tAcquiring";
        break;

      case DHTLIB_ERROR_DELTA:
        Serial.println("Error\n\r\tDelta time to small");
        sensorStatus = "Error\n\r\tDelta time to small";
        break;

      case DHTLIB_ERROR_NOTSTARTED:
        Serial.println("Error\n\r\tNot started");
        sensorStatus = "Error\n\r\tNot started";
        break;

      default:
        Serial.println("Unknown error");
        sensorStatus = "Unknown error";
        break;

      }

    Serial.print("Humidity (%): ");
    Serial.println(DHT.getHumidity(), 2);
    Particle.publish("Humidity (%)", String(DHT.getHumidity(), 2));
    dHumidity = (double) DHT.getHumidity();

    Serial.print("Temperature (°C): ");
    Serial.println(DHT.getCelsius(), 2);
    Particle.publish("Temperature (°C)", String(DHT.getCelsius(), 2));
    dTemperature = (double) DHT.getCelsius();

    //Serial.print("Temperature (°F): ");
    //Serial.println(DHT.getFahrenheit(), 2);

    //Serial.print("Temperature (K): ");
    //Serial.println(DHT.getKelvin(), 2);

    Serial.print("Dew Point (°C): ");
    Serial.println(DHT.getDewPoint());

    Serial.print("Dew Point Slow (°C): ");
    Serial.println(DHT.getDewPointSlow());

    n++;  // increment counter
    bDHTstarted = false;  // reset the sample flag so we can take another
    DHTnextSampleTime = millis() + DHT_SAMPLE_INTERVAL;  // set the time for next sample

    }

  }

}


// loop
void loop() {

  // read temperature and humidity of sensor
  // readTempHumid();

  // trigger garage door
  //digitalWrite(pushButton, HIGH);          // sets the LED on
  //delay(300);                       // waits for 200mS
  //digitalWrite(pushButton, LOW);           // sets the LED off
  //delay(10000);                       // waits for 200mS

  // trigger echo measurement
  // Trigger pin, Echo pin, delay (ms), visual=true|info=false
  //ping(D5, D6, mm, true);

  //Serial.printf("Distance: %i mm", ping(D5, D6, 3, true));
  //Serial.println();

  Serial.printf("Distance 1: %i mm", measureDistance(1));
  Serial.println();
  Serial.printf("Distance 2: %i mm", measureDistance(2));
  Serial.println();

  delay(500); // slow down the output


}

int measureDistance(int sensorNumber) {

  switch (sensorNumber) {

    case 1:
      // measurement of sensor 1
      return ping(D5, D6, 3, true);
      break;

    case 2:
      // measurement of sensor 1
      return ping(D3, D4, 3, true);
      break;

    default:
      return 0;

  }

}


int ping(pin_t trig_pin, pin_t echo_pin, int unit, bool info) {

    //uint32_t speedOfSound, duration, inches, cm, meter;

    uint32_t mm, cm, meter, timeMeasurement, timeDistance;
    float speedOfSound = 331.5 + 0.6 * 22; // 22 °C

    //static bool init = false;

    //if (!init) {

        // setting trigger pin
        pinMode(trig_pin, OUTPUT);
        digitalWriteFast(trig_pin, LOW);

        // setting echo pin
        pinMode(echo_pin, INPUT);
        delay(50);
        //init = true;

    //}

    // trigger the sensor by sending a HIGH pulse of 10 or more microseconds
    digitalWriteFast(trig_pin, HIGH);
    delayMicroseconds(10);
    digitalWriteFast(trig_pin, LOW);

    // wait for response
    timeMeasurement = pulseIn(echo_pin, HIGH);
    timeDistance = timeMeasurement / 2;


    // Ausbreitungsgeschwindigkeit (in Luft) = 331,5 + (0,6 * Temp°C) speedOfSound = 331,5 + ( 0.6 * dTemperature);

    // Convert the time into a distance
    // Sound travels at 1130 ft/s (73.746 µs/inch)
    // or 340 m/s (29 µs/cm), out and back so divide by 2
    // Ref: http://www.parallax.com/dl/docs/prod/acc/28015-PING-v1.3.pdf
    //inches = duration / 74 / 2;
    //cm = duration / 29 / 2;
    //meter = duration / 0.034 / 2;

    meter = speedOfSound * timeDistance * pow(10, -6);
    cm = speedOfSound * timeDistance * pow(10,-4);
    mm = speedOfSound * timeDistance * pow(10,-3);

    if (info) { // Visual Output

        //Serial.printf("%2d:", inches);
        //for(int x=0;x<inches;x++) Serial.print("#");
        //Serial.println();

    } else { // Informational Output

        // Serial.printlnf("%6d in / %6d cm / %6d µs / %6d v", inches, cm, duration, speedOfSound);
        //Serial.printlnf("Distance: %s", cmDistance.toString().c_str());

        Serial.printf("SoS: %f m/s", speedOfSound);
        Serial.println();
        Serial.printf("Distance: %i m", meter);
        Serial.println();
        Serial.printf("Distance: %i cm", cm);
        Serial.println();
        Serial.printf("Distance: %i mm", mm);
        Serial.println();

    }



    switch (unit) {

      case 1:
        // statements
        return meter;
        break;

      case 2:
        // statements
        return cm;
        break;

      case 3:
        // statements
        return mm;
        break;

      default:
        // statements
        return meter;
    }


}
