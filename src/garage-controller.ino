/*******************************************************************************
 * Project  : GarageController
 * Compiler : Particle Photon
 * Verion   : 0.8.0-rc.10
 * Date     : 15.10.2018
 * Author   : Tim Hornikel
 * License  : GNU General Public License v3+
*******************************************************************************/

// includings of 3rd party libraries
// @todo: check necessary includings
#include "application.h"
#include "PietteTech_DHT.h" // Temperature, Humidity Sensor Libray
#include "math.h"

 // system defines
 // @todo: check necessary defines
#define DHTTYPE  DHT22              // Sensor type DHT11/21/22/AM2301/AM2302
#define DHTPIN   D3           	    // Digital pin for communications
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
//STARTUP(WiFi.selectAntenna(ANT_EXTERNAL));

// globals
int DHTnextSampleTime;	    // Next time we want to start sample
bool bDHTstarted;		    // flag to indicate we started acquisition
int n;                              // counter
String sensorStatus = "Initialization";
// Version MAJOR.MINOR.PATCH
String firmwareVersion = "0.1.0";
double dHumidity = 0.0;
double dTemperature = 0.0;


const int sensorHight1 = 75;  // hight of mounted sensor 1 in cm
const int sensorHight2 = 75;  // hight of mounted sensor 2 in cm
const int vehicleHight = 30;  // hight of vehicle to detect in cm

bool sensorDetect1 = false;   // status of detected vehicle for sensor 1
bool sensorDetect2 = false;   // status of detected vehicle for sensor 2

int vehicleInGarage = 1;

// 1 = inside garage
// 2 = transition
// 3 = outside in garage

// declaration of I/Os
int garageTrigger = D0;       // garage door trigger relay
int usTrigger = D4;   // trigger signal for ultrasonic sensors
int usEcho1 = D5;     // echo signal for ultrasonic sensor 1
int usEcho2 = D6;     // echo signal for ultrasonic sensor 2
int statusLED = D7;   // status LED for signalling if vehcile detected with sensor 1
int statusLEDParticle = D1;

// Declaration of time variables
unsigned long lastSync = millis();                    // last synchronization of time in internet
unsigned long ONE_DAY_MILLIS (24 * 60 * 60 * 1000);   // 24 hours
unsigned long lastNotification = millis();            //
unsigned long threshold (15 * 60 * 1000);             // 15 minutes
unsigned long lastCheck = millis();                   //
unsigned long thresholdCheck (5 * 1000); // 5 seconds                    // one ultrasonic measurement every second


// Setup
void setup() {

  // serial communications
  Serial.begin(9600);
  // Wait for a USB serial connection for up to 3 seconds
  //waitFor(Serial.isConnected, 3000);
  Serial.printlnf("System version: %s", System.version().c_str());
  Serial.printlnf("Firmware version: %s", firmwareVersion.c_str());
  Serial.println("---------------");

  // particle variables
  Particle.variable("Status", sensorStatus);
  Particle.variable("Temperature", &dTemperature, DOUBLE);
  Particle.variable("Humidity", &dHumidity, DOUBLE);

  // Definition of I/O-Pins
  pinMode(garageTrigger, OUTPUT);
  pinMode(statusLED, OUTPUT);
  pinMode(statusLEDParticle, OUTPUT);

  // setting trigger pin
  pinMode(usTrigger, OUTPUT);
  digitalWriteFast(usTrigger, LOW);

  // setting echo pins
  pinMode(usEcho1, INPUT);
  pinMode(usEcho2, INPUT);

  DHTnextSampleTime = 0;  // Start the first sample immediately

  digitalWrite(garageTrigger, LOW);

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

void triggerGarage() {

  digitalWrite(garageTrigger, HIGH);
  delay(400);
  digitalWrite(garageTrigger, LOW);

}


void takeMeasurements() {

  if (detectVehicle(1) == true) {

    //Serial.println("Vehicle detected by sensor 1");
    digitalWrite(statusLED, HIGH);
    sensorDetect1 = true;

  } else {

    digitalWrite(statusLED, LOW);
    sensorDetect1 = false;

  }

  if (detectVehicle(2) == true) {

    //Serial.println("Vehicle detected by sensor 2");
    sensorDetect2 = true;

  } else {

    sensorDetect2 = false;

  }

  delay(500);

}





// loop
void loop() {

  // read temperature and humidity of sensor
  readTempHumid();

  takeMeasurements();

  if (sensorDetect1 == true && sensorDetect2 == false && vehicleInGarage > 1) {
    // vehicle inside

      //triggerGarage();
      Serial.println("vehicle inside");
      vehicleInGarage--;

  } else if (sensorDetect1 == false && sensorDetect2 == true && vehicleInGarage < 2) {
    // vehicle in transition

      Serial.println("vehicle in transition");
      vehicleInGarage++;

  } else if (sensorDetect1 == false && sensorDetect2 == true && vehicleInGarage > 2) {
    // vehicle in transition

      Serial.println("vehicle in transition");
      vehicleInGarage--;

  } else if (sensorDetect1 == false && sensorDetect2 == false && vehicleInGarage < 3) {
    // vehicle outside

    triggerGarage();
    Serial.println("vehicle outside");
    vehicleInGarage++;

  } else {


  }

  updateTime();

  checkStatus();





}

/*******************************************************************************
 * Function Name  : CheckStatus
 * Description    : Checks if Particle cloud is connected
 * Input          : None
 * Output         : Blue flashing LED every 5 seconds
 * Return         : None
 *******************************************************************************/
void checkStatus() {

  if ((Particle.connected() == true) && (millis() - lastCheck > thresholdCheck)) {

    digitalWrite(statusLEDParticle, HIGH);
    delay(100);
    digitalWrite(statusLEDParticle, LOW);
    lastCheck = millis();

  }

}


/*******************************************************************************
 * Function Name  : updateTime
 * Description    : Request time synchronization from the Particle Cloud once a day
 * Input          : None
 * Output         : None
 * Return         : None
 *******************************************************************************/
void updateTime() {

    if (millis() - lastSync > ONE_DAY_MILLIS) {

    // Request time synchronization from the Particle Cloud
    Particle.syncTime();
    Serial.printf("Time updated at %s...", Time.timeStr().c_str());
    lastSync = millis();

    }

}


bool detectVehicle(int sensorNumber) {

  int distance;

  if (sensorNumber == 1) {

    distance = sensorHight1 - measureDistance(sensorNumber);


  } else if (sensorNumber == 2) {

    distance = sensorHight2 - measureDistance(sensorNumber);

  }

  if (distance >= vehicleHight) {

    return true;

  } else {

    return false;

  }

}


int measureDistance(int sensorNumber) {

  uint32_t distance, timeMeasurement, timeDistance;

  // Ausbreitungsgeschwindigkeit (in Luft) = 331,5 + (0,6 * Temp°C) speedOfSound = 331,5 + ( 0.6 * dTemperature);
  float speedOfSound = 331.5 + 0.6 * 22; // 22 °C

  // trigger the sensor by sending a HIGH pulse of 10 or more microseconds
  digitalWrite(usTrigger, LOW);
  delayMicroseconds(3);

  // critical, time-sensitive code starts
  noInterrupts();

  digitalWriteFast(usTrigger, HIGH);
  delayMicroseconds(10);
  digitalWriteFast(usTrigger, LOW);

  if (sensorNumber == 1) {

    timeMeasurement = pulseIn(usEcho1, HIGH);

  } else if (sensorNumber == 2) {

    timeMeasurement = pulseIn(usEcho2, HIGH);

  }

  // critical, time-sensitive code ends
  interrupts();

  timeDistance = timeMeasurement / 2;
  distance = speedOfSound * timeDistance * pow(10,-4); // in cm


  // calclulate distance in mm
  return distance;
  delay(50);

}
