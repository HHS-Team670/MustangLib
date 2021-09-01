// /**
//  * TCA9548 I2CScanner.ino -- I2C bus scanner for Arduino
//  *
//  * Based on https://playground.arduino.cc/Main/I2cScanner/
//  * MAY NOT NEED
//  */

// #include "Wire.h"

// #define TCAADDR 0x70

// void tcaselect(uint8_t i) {
//   if (i > 7) return;
 
//   Wire.beginTransmission(TCAADDR);
//   Wire.write(1 << i);
//   Wire.endTransmission();  
// }


// // standard Arduino setup()
// void setup()
// {
//     while (!Serial);
//     delay(1000);

//     Wire.begin();
    
//     Serial.begin(115200);
//     Serial.println("\nTCAScanner ready!");
    
//     for (uint8_t t=0; t<8; t++) {
//       tcaselect(t);
//       Serial.print("TCA Port #"); Serial.println(t);

//       for (uint8_t addr = 0; addr<=127; addr++) {
//         if (addr == TCAADDR) continue;

//         Wire.beginTransmission(addr);
//         if (!Wire.endTransmission()) {
//           Serial.print("Found I2C 0x");  Serial.println(addr,HEX);
//         }
//       }
//     }
//     Serial.println("\ndone");
// }

// void loop() {}


//-------------------

//imports
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>

//variable
#define TCAADDR 0x70

/* Assign a unique ID to this sensor at the same time */
Adafruit_HMC5883_Unified mag1 = Adafruit_HMC5883_Unified(1);
Adafruit_HMC5883_Unified mag2 = Adafruit_HMC5883_Unified(2);

//random printing for sensor details
private displaySensorDetails(param)//not sure what to put here)
{
  SomeSensorClass sensor;

  param = getSensor() //some other thing I don't get)
  
}
// void displaySensorDetails(Adafruit_HMC5883_Unified *mag)
// {
//   sensor_t sensor;
//   mag->getSensor(&sensor);
//   Serial.println("------------------------------------");
//   Serial.print  ("Sensor:       "); Serial.println(sensor.name);
//   Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
//   Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
//   Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" uT");
//   Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" uT");
//   Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" uT");  
//   Serial.println("------------------------------------");
//   Serial.println("");
//   delay(500);
// }

void tcaselect(uint8_t i) {
  if (i > 7) return;
 
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();  
}


void setup(void) 
{
  Serial.begin(9600);
  Serial.println("HMC5883 Magnetometer Test"); Serial.println("");
  
  /* Initialise the 1st sensor */
  tcaselect(2);
  if(!mag1.begin())
  {
    /* There was a problem detecting the HMC5883 ... check your connections */
    Serial.println("Ooops, no HMC5883 detected ... Check your wiring!");
    while(1);
  }
  
  /* Initialise the 2nd sensor */
  tcaselect(6);
  if(!mag2.begin())
  {
    /* There was a problem detecting the HMC5883 ... check your connections */
    Serial.println("Ooops, no HMC5883 detected ... Check your wiring!");
    while(1);
  }
  
  /* Display some basic information on this sensor */
  tcaselect(2);
  displaySensorDetails(&mag1);
  tcaselect(6);
  displaySensorDetails(&mag2);
}

void loop(void) 
{
  /* Get a new sensor event */ 
  sensors_event_t event; 
  
  tcaselect(2);
  mag1.getEvent(&event);
 
  /* Display the results (magnetic vector values are in micro-Tesla (uT)) */
  Serial.print("Sensor #1 - ");
  Serial.print("X: "); Serial.print(event.magnetic.x); Serial.print("  ");
  Serial.print("Y: "); Serial.print(event.magnetic.y); Serial.print("  ");
  Serial.print("Z: "); Serial.print(event.magnetic.z); Serial.print("  ");Serial.println("uT");
  
  tcaselect(6);
  mag2.getEvent(&event);
  /* Display the results (magnetic vector values are in micro-Tesla (uT)) */
  Serial.print("Sensor #2 - ");
  Serial.print("X: "); Serial.print(event.magnetic.x); Serial.print("  ");
  Serial.print("Y: "); Serial.print(event.magnetic.y); Serial.print("  ");
  Serial.print("Z: "); Serial.print(event.magnetic.z); Serial.print("  ");Serial.println("uT");
  
  delay(500);
}