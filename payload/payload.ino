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

SHT1x sht1x(dataPin, clockPin);

TinyGPS gps;


//Digital Pins
SoftwareSerial ss(2, 10); //GPS Pins. 10 is declared, but not used

//Pins for the accelerometer
const int xPin = 0;
const int yPin = 1;
const int zPin = 2;

//Memory addresses for the Temperature and Pressure sensor
#define BMP085_ADDRESS 0x77

static void smartdelay(unsigned long ms);
static void print_float(float val, float invalid, int len, int prec);
static void print_int(unsigned long val, unsigned long invalid, int len);
static void print_date(TinyGPS &gps);
static void print_str(const char *str, int len);








//The minimum and maximum values that came from
//the accelerometer while standing still
//You very well may need to change these
int minVal = 265;
int maxVal = 402;


//to hold the caculated values
double x;
double y;
double z;


const unsigned char OSS = 0;  // Oversampling Setting

// Calibration values
int ac1;
int ac2;
int ac3;
unsigned int ac4;
unsigned int ac5;
unsigned int ac6;
int b1;
int b2;
int mb;
int mc;
int md;

// b5 is calculated in bmp085GetTemperature(...), this variable is also used in bmp085GetPressure(...)
// so ...Temperature(...) must be called before ...Pressure(...).
long b5;

short temperature;
long pressure;



// Read the uncompensated temperature value
unsigned int bmp085ReadUT() {
    unsigned int ut;

    // Write 0x2E into Register 0xF4
    // This requests a temperature reading
    Wire.beginTransmission(BMP085_ADDRESS);
    Wire.write(0xF4);
    Wire.write(0x2E);
    Wire.endTransmission();

    // Wait at least 4.5ms
    delay(5);

    // Read two bytes from registers 0xF6 and 0xF7
    ut = bmp085ReadInt(0xF6);
    return ut;
}

// Read the uncompensated pressure value
unsigned long bmp085ReadUP() {
    unsigned char msb, lsb, xlsb;
    unsigned long up = 0;

    // Write 0x34+(OSS<<6) into register 0xF4
    // Request a pressure reading w/ oversampling setting
    Wire.beginTransmission(BMP085_ADDRESS);
    Wire.write(0xF4);
    Wire.write(0x34 + (OSS << 6));
    Wire.endTransmission();

    // Wait for conversion, delay time dependent on OSS
    delay(2 + (3 << OSS));

    // Read register 0xF6 (MSB), 0xF7 (LSB), and 0xF8 (XLSB)
    Wire.beginTransmission(BMP085_ADDRESS);
    Wire.write(0xF6);
    Wire.endTransmission();
    Wire.requestFrom(BMP085_ADDRESS, 3);

    // Wait for data to become available
    while (Wire.available() < 3)
        ;
    msb = Wire.read();
    lsb = Wire.read();
    xlsb = Wire.read();

    up = (((unsigned long) msb << 16) | ((unsigned long) lsb << 8) | (unsigned long) xlsb) >> (8 - OSS);

    return up;
}

// Calculate temperature given ut.
// Value returned will be in units of 0.1 deg C
short bmp085GetTemperature(unsigned int ut) {
    long x1, x2;

    x1 = (((long)ut - (long)ac6) * (long)ac5) >> 15;
    x2 = ((long)mc << 11) / (x1 + md);
    b5 = x1 + x2;

    return ((b5 + 8) >> 4);
}

// Calculate pressure given up
// calibration values must be known
// b5 is also required so bmp085GetTemperature(...) must be called first.
// Value returned will be pressure in units of Pa.
float bmp085GetPressure(unsigned long up) {
    long x1, x2, x3, b3, b6, p;
    unsigned long b4, b7;

    b6 = b5 - 4000;
    // Calculate B3
    x1 = (b2 * (b6 * b6) >> 12) >> 11;
    x2 = (ac2 * b6) >> 11;
    x3 = x1 + x2;
    b3 = (((((long)ac1) * 4 + x3) << OSS) + 2) >> 2;

    // Calculate B4
    x1 = (ac3 * b6) >> 13;
    x2 = (b1 * ((b6 * b6) >> 12)) >> 16;
    x3 = ((x1 + x2) + 2) >> 2;
    b4 = (ac4 * (unsigned long)(x3 + 32768)) >> 15;

    b7 = ((unsigned long)(up - b3) * (50000 >> OSS));
    if (b7 < 0x80000000)
        p = (b7 << 1) / b4;
    else
        p = (b7 / b4) << 1;

    x1 = (p >> 8) * (p >> 8);
    x1 = (x1 * 3038) >> 16;
    x2 = (-7357 * p) >> 16;
    p += (x1 + x2 + 3791) >> 4;


    return (p / 1000) * .009;


}

char bmp085Read(unsigned char address) {
    unsigned char data;

    Wire.beginTransmission(BMP085_ADDRESS);
    Wire.write(address);
    Wire.endTransmission();

    Wire.requestFrom(BMP085_ADDRESS, 1);
    while (!Wire.available())
        ;

    return Wire.read();
}

int bmp085ReadInt(unsigned char address) {
    unsigned char msb, lsb;

    Wire.beginTransmission(BMP085_ADDRESS);
    Wire.write(address);
    Wire.endTransmission();

    Wire.requestFrom(BMP085_ADDRESS, 2);
    while (Wire.available() < 2)
        ;
    msb = Wire.read();
    lsb = Wire.read();

    return (int) msb << 8 | lsb;
}

static void smartdelay(unsigned long ms) {
    unsigned long start = millis();
    do {
        while (ss.available())
            gps.encode(ss.read());
    } while (millis() - start < ms);
}

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

    float flat, flon;
    unsigned long age, date, time, chars = 0;
    unsigned short sentences = 0, failed = 0;
    static const double LONDON_LAT = 42.729159, LONDON_LON = -73.674154;

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
