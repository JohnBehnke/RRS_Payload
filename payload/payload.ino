//////////////////////////////////////////////////////////////////
//Some code ©2011 bildr
//Released under the MIT License - Please reuse change and share
//Simple code for the ADXL335, prints calculated orientation via serial
//Adapted for the Rensselaer Rocket Society's payload by Brian Kelley and John Behnke
//////////////////////////////////////////////////////////////////

#include <TinyGPS.h>
#include <SoftwareSerial.h>
#include <Wire.h>

// Library: https://github.com/practicalarduino/SHT1x
#include <SHT1x.h>

#define dataPin  10
#define clockPin 11




return (p / 1000) * .009;












static void print_float(float val, float invalid, int len, int prec) {
    if (val == invalid) {
        while (len-- > 1)
            Serial.print('*');
        Serial.print(' ');
    } else {
        Serial.print(val, prec);
        int vi = abs((int)val);
        int flen = prec + (val < 0.0 ? 2 : 1); // . and -
        flen += vi >= 1000 ? 4 : vi >= 100 ? 3 : vi >= 10 ? 2 : 1;
        for (int i = flen; i < len; ++i)
            Serial.print(' ');
    }
    smartdelay(0);
}

static void print_int(unsigned long val, unsigned long invalid, int len) {
    char sz[32];
    if (val == invalid)
        strcpy(sz, "*******");
    else
        sprintf(sz, "%ld", val);
    sz[len] = 0;
    for (int i = strlen(sz); i < len; ++i)
        sz[i] = ' ';
    if (len > 0)
        sz[len - 1] = ' ';
    Serial.print(sz);
    smartdelay(0);
}

static void print_date(TinyGPS &gps) {
    int year;
    byte month, day, hour, minute, second, hundredths;
    unsigned long age;
    gps.crack_datetime(&year, &month, &day, &hour, &minute, &second, &hundredths, &age);
    if (age == TinyGPS::GPS_INVALID_AGE)
        Serial.print("********** ******** ");
    else {
        char sz[32];
        sprintf(sz, "%02d/%02d/%02d %02d:%02d:%02d ",
                month, day, year, hour, minute, second);
        Serial.print(sz);
    }
    print_int(age, TinyGPS::GPS_INVALID_AGE, 5);
    smartdelay(0);
}

static void print_str(const char *str, int len) {
    int slen = strlen(str);
    for (int i = 0; i < len; ++i)
        Serial.print(i < slen ? str[i] : ' ');
    smartdelay(0);
}

void setup() {
    Serial.begin(57600);
    ss.begin(57600);
    Wire.begin();
    bmp085Calibration();
    //Grabs the time since the start of the arduino in milliseconds.
    time_since_start = millis();
}

void loop() {

    //read the analog values from the accelerometer
    //int xRead = analogRead(xPin);
    //int yRead = analogRead(yPin);
    //int zRead = analogRead(zPin);

    //convert read values to degrees -90 to 90 - Needed for atan2
    //int xAng = map(xRead, minVal, maxVal, -90, 90);
    //int yAng = map(yRead, minVal, maxVal, -90, 90);
    //int zAng = map(zRead, minVal, maxVal, -90, 90);



    //Caculate 360deg values like so: atan2(-yAng, -zAng)
    //atan2 outputs the value of -π to π (radians)
    //We are then converting the radians to degrees
    //x = RAD_TO_DEG * (atan2(-yAng, -zAng) + PI);
    //y = RAD_TO_DEG * (atan2(-xAng, -zAng) + PI);
    //z = RAD_TO_DEG * (atan2(-yAng, -xAng) + PI);

    float pressure = bmp085GetPressure(bmp085ReadUP());
    gps.f_get_position(&flat, &flon, &age);
    float temperature = sht1x.readTemperatureC();
    float humidity = sht1x.readHumidity();

    Serial.print("OK|");
    //Serial.print("|");
    Serial.print(temperature);
    //  Serial.println(" deg C");
    Serial.print("|");
    Serial.print(pressure, DEC);
    //  Serial.println(" Pa");

    Serial.print("|");
    Serial.print(humidity);
    Serial.print("|");
    print_float(flat, TinyGPS::GPS_INVALID_F_ANGLE, 10, 6);
    Serial.print("|");
    print_float(flon, TinyGPS::GPS_INVALID_F_ANGLE, 11, 6);
    Serial.print("|");
    print_float(gps.f_altitude(), TinyGPS::GPS_INVALID_F_ALTITUDE, 7, 2);
    Serial.print("|");
    Serial.print(time_since_start)
    Serial.print("|OK");
    Serial.println();


    //smartdelay(100);//just here to slow down the serial output - Easier to read
}
