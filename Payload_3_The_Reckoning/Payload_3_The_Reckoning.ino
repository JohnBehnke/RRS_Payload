#include <Wire.h>
#include <SD.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_BMP085_U.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_10DOF.h>

/* Assign a unique ID to the sensors */
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(30302);
Adafruit_BMP085_Unified       bmp   = Adafruit_BMP085_Unified(18001);
Adafruit_L3GD20_Unified       gyro  = Adafruit_L3GD20_Unified(20);

// Used for storing the data we collect.
File dataLog;
char fileName[] = "log.txt";

void setup(void) {
    Serial.begin(9600);
    pinMode(SS, OUTPUT);
    /* Initialise the sensors */

    if (!accel.begin()) {
        /* There was a problem detecting the ADXL345 ... check your connections */
        Serial.println(F("Ooops, no LSM303 detected ... Check your wiring!"));
        while (1);
    }
    if (!mag.begin()) {
        /* There was a problem detecting the LSM303 ... check your connections */
        Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
        while (1);
    }
    if (!bmp.begin()) {
        /* There was a problem detecting the BMP085 ... check your connections */
        Serial.print("Ooops, no BMP085 detected ... Check your wiring or I2C ADDR!");
        while (1);
    }
    if (!gyro.begin()) {
        /* There was a problem detecting the L3GD20 ... check your connections */
        Serial.print("Ooops, no L3GD20 detected ... Check your wiring or I2C ADDR!");
        while (1);
    }

    if (!SD.begin(10)) {
        Serial.println("Initialization of SD card failed!");
        while (1);
    }
    
    if (SD.exists(fileName)) {
      SD.remove(fileName);
    }


    dataLog = SD.open( fileName , FILE_WRITE);

    if (dataLog) {
        Serial.println("Opened file correctly!");

    } else {
        Serial.println("Error opening file");
    }

    if (SD.exists(fileName)) {
        Serial.println("file exists.");
    } else {
        Serial.println("file doesn't exist.");
    }

}

void loop(void) {
    /* Get a new sensor event */
    sensors_event_t event;

    // prints out: X Accelerometer data | Magnetometer data | gyroscopic data | pressure data Z
    //                    X Y Z       |       X Y Z       |       X Y Z     | PRESSURE TEMPERATURE ALTITUDE Z

    /* Display the results (acceleration is measured in m/s^2) */
    accel.getEvent(&event);
    //Serial.print(F("ACCEL "));
    //Serial.print("X: ");
    Serial.print(event.acceleration.x);
    Serial.print(" ");
    //Serial.print("Y: ");
    Serial.print(event.acceleration.y);
    Serial.print(" ");
    //Serial.print("Z: ");
    Serial.print(event.acceleration.z);
    Serial.print(" | ");
    //Serial.println("m/s^2 ");

    //dataLog.print(F("ACCEL "));
    //dataLog.print("X: ");
    dataLog.print(event.acceleration.x);
    dataLog.print(" ");
    //dataLog.print("Y: ");
    dataLog.print(event.acceleration.y);
    dataLog.print(" ");
    //dataLog.print("Z: ");
    dataLog.print(event.acceleration.z);
    dataLog.print(" | ");
    //dataLog.println("m/s^2 ");

    /* Display the results (magnetic vector values are in micro-Tesla (uT)) */
    mag.getEvent(&event);
    //Serial.print(F("MAG   "));
    //Serial.print("X: ");
    Serial.print(event.magnetic.x);
    Serial.print(" ");
    //Serial.print("Y: ");
    Serial.print(event.magnetic.y);
    Serial.print(" ");
    //Serial.print("Z: ");
    Serial.print(event.magnetic.z);
    Serial.print(" | ");
    //Serial.println("uT");

    //dataLog.print(F("MAG   "));
    //dataLog.print("X: ");
    dataLog.print(event.magnetic.x);
    dataLog.print(" ");
    //dataLog.print("Y: ");
    dataLog.print(event.magnetic.y);
    dataLog.print(" ");
    //dataLog.print("Z: ");
    dataLog.print(event.magnetic.z);
    dataLog.print(" | ");
    //dataLog.println("uT");

    /* Display the results (gyrocope values in rad/s) */
    gyro.getEvent(&event);
    //Serial.print(F("GYRO  "));
    //Serial.print("X: ");
    Serial.print(event.gyro.x);
    Serial.print(" ");
    //Serial.print("Y: ");
    Serial.print(event.gyro.y);
    Serial.print(" ");
    //Serial.print("Z: ");
    Serial.print(event.gyro.z);
    Serial.print(" | ");
    //Serial.println("rad/s ");

    //dataLog.print(F("GYRO  "));
    //dataLog.print("X: ");
    dataLog.print(event.gyro.x);
    dataLog.print(" ");
    //dataLog.print("Y: ");
    dataLog.print(event.gyro.y);
    dataLog.print(" ");
    //dataLog.print("Z: ");
    dataLog.print(event.gyro.z);
    dataLog.print(" | ");
    //dataLog.println("rad/s ");

    /* Display the pressure sensor results (barometric pressure is measure in hPa) */
    bmp.getEvent(&event);
    if (event.pressure) {
        /* Display atmospheric pressure in hPa */
        //Serial.print(F("PRESS "));
        Serial.print(event.pressure);
        Serial.print(F(" "));

        //dataLog.print(F("PRESS "));
        dataLog.print(event.pressure);
        dataLog.print(F(" "));

        /* Display ambient temperature in C */
        float temperature;
        bmp.getTemperature(&temperature);
        Serial.print(temperature);
        Serial.print(F(" "));

        dataLog.print(temperature);
        dataLog.print(F(" "));

        /* Then convert the atmospheric pressure, SLP and temp to altitude    */
        /* Update this next line with the current SLP for better results      */
        float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;
        Serial.print(bmp.pressureToAltitude(seaLevelPressure,
                                            event.pressure,
                                            temperature));
        Serial.println(F(" "));

        dataLog.print(bmp.pressureToAltitude(seaLevelPressure,
                                             event.pressure,
                                             temperature));
        dataLog.println(F(" "));
    }

    //Serial.println(F(""));
    //dataLog.println(F(""));
    dataLog.flush();
}




