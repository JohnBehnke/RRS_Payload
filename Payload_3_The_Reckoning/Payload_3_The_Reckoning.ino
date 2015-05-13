#include <SoftwareSerial.h>
#include <Adafruit_GPS.h>
#include <Wire.h>
#include <SD.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_BMP085_U.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_10DOF.h>

/* Assign a unique ID to the sensors */
Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(30302);
Adafruit_BMP085_Unified       bmp   = Adafruit_BMP085_Unified(18001);
Adafruit_L3GD20_Unified       gyro  = Adafruit_L3GD20_Unified(20);

// Used for storing the data we collect.
File dataLog;
char fileName[] = "log.txt";

void setup(void) {
    Serial.begin(9600);
    pinMode(SS, OUTPUT);



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
    // accelX|accelY|accelZ|magX|magY|magZ|gyroX|gyroY|gyroZ|pressure|temperature|altitude

    /* Display the results (acceleration is measured in m/s^2) */

    float start = millis();

    /* Display the results (magnetic vector values are in micro-Tesla (uT)) */
    mag.getEvent(&event);
    //Serial.print(F("MAG   "));
    //Serial.print("X: ");
    Serial.print(event.magnetic.x);
    Serial.print("|");
    //Serial.print("Y: ");
    Serial.print(event.magnetic.y);
    Serial.print("|");
    //Serial.print("Z: ");
    Serial.print(event.magnetic.z);
    Serial.print("|");
    //Serial.println("uT");

    //dataLog.print(F("MAG   "));
    //dataLog.print("X: ");
    dataLog.print(event.magnetic.x);
    dataLog.print("|");
    //dataLog.print("Y: ");
    dataLog.print(event.magnetic.y);
    dataLog.print("|");
    //dataLog.print("Z: ");
    dataLog.print(event.magnetic.z);
    dataLog.print("|");
    //dataLog.println("uT");

    /* Display the results (gyrocope values in rad/s) */
    gyro.getEvent(&event);
    //Serial.print(F("GYRO  "));
    //Serial.print("X: ");
    Serial.print(event.gyro.x);
    Serial.print("|");
    //Serial.print("Y: ");
    Serial.print(event.gyro.y);
    Serial.print("|");
    //Serial.print("Z: ");
    Serial.print(event.gyro.z);
    Serial.print("|");
    //Serial.println("rad/s ");

    //dataLog.print(F("GYRO  "));
    //dataLog.print("X: ");
    dataLog.print(event.gyro.x);
    dataLog.print("|");
    //dataLog.print("Y: ");
    dataLog.print(event.gyro.y);
    dataLog.print("|");
    //dataLog.print("Z: ");
    dataLog.print(event.gyro.z);
    dataLog.print("|");
    //dataLog.println("rad/s ");

    /* Display the pressure sensor results (barometric pressure is measure in hPa) */
    bmp.getEvent(&event);
    if (event.pressure) {

        /* Display ambient temperature in C */
        float temperatureC;


        bmp.getTemperature(&temperatureC);

        //float temperatureF = (temperatureC * 9 * 5) + 32; //F for easy debugging

        Serial.print(temperatureC);
        Serial.print("|");

        dataLog.print(temperatureC);
        dataLog.print("|");
    }

    float finished = millis();

    Serial.print(finished - start);
    dataLog.print(finished - start);


    Serial.println("");
    dataLog.println("");
    dataLog.flush();
}




