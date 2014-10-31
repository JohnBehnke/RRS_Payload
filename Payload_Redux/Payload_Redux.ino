

// configure LED for output
pinMode(LED_PIN, OUTPUT);
}


// ================================================================
// ===                        ------------                      ===
// ================================================================

void loop() {

    // ================================================================
    // ===                       SHT15 Main Code                    ===
    // ================================================================



    // read the temperature
    sendCommandSHT(temperatureCommand, dataPin, clockPin);
    waitForResultSHT(dataPin);
    val = getData16SHT(dataPin, clockPin);
    skipCrcSHT(dataPin, clockPin);
    temperature = (float)val * 0.01 - 40;
    temperature = (float)temperature * 9 / 5 + 32; //Convert from Celsius to Fahrenheit

    // read the humidity
    sendCommandSHT(humidityCommand, dataPin, clockPin);
    waitForResultSHT(dataPin);
    val = getData16SHT(dataPin, clockPin);
    skipCrcSHT(dataPin, clockPin);
    humidity = -4.0 + 0.0405 * val + -0.0000028 * val * val;

    // output readings to serial port
    Serial.print((temperature));
    Serial.print("        ");
    Serial.println(humidity);
    // wait for 1 second for next reading

    // ================================================================
    // ===                     BMP085 Main Code                     ===
    // ================================================================
    pressure = bmp085GetPressure(bmp085ReadUP());
    altitude = (float)44330 * (1 - pow(((float) pressure / p0), 0.190295));


    Serial.print("Pressure: ");
    Serial.print(pressure, DEC);
    Serial.println(" Pa");
    Serial.print("Altitude: ");
    Serial.print(altitude, 2);
    Serial.println(" m");
    Serial.println("________________");
    //delay(1000);

    // ================================================================
    // ===                    MPU6050 Main LOOP                     ===
    // ================================================================

    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        //reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));

        // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);

        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

#ifdef OUTPUT_READABLE_QUATERNION
        // display quaternion values in easy matrix form: w x y z
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        Serial.print("quat\t");
        Serial.print(q.w);
        Serial.print("\t");
        Serial.print(q.x);
        Serial.print("\t");
        Serial.print(q.y);
        Serial.print("\t");
        Serial.println(q.z);
#endif

#ifdef OUTPUT_READABLE_EULER
        // display Euler angles in degrees
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetEuler(euler, &q);
        Serial.print("euler\t");
        Serial.print(euler[0] * 180 / M_PI);
        Serial.print("\t");
        Serial.print(euler[1] * 180 / M_PI);
        Serial.print("\t");
        Serial.println(euler[2] * 180 / M_PI);
#endif

#ifdef OUTPUT_READABLE_YAWPITCHROLL
        // display Euler angles in degrees
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        Serial.print("ypr\t");
        Serial.print(ypr[0] * 180 / M_PI);
        Serial.print("\t");
        Serial.print(ypr[1] * 180 / M_PI);
        Serial.print("\t");
        Serial.println(ypr[2] * 180 / M_PI);
#endif

#ifdef OUTPUT_READABLE_REALACCEL
        // display real acceleration, adjusted to remove gravity
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetAccel(&aa, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
        Serial.print("areal\t");
        Serial.print(aaReal.x);
        Serial.print("\t");
        Serial.print(aaReal.y);
        Serial.print("\t");
        Serial.println(aaReal.z);
#endif

#ifdef OUTPUT_READABLE_WORLDACCEL
        // display initial world-frame acceleration, adjusted to remove gravity
        // and rotated based on known orientation from quaternion
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetAccel(&aa, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
        mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
        Serial.print("aworld\t");
        Serial.print(aaWorld.x);
        Serial.print("\t");
        Serial.print(aaWorld.y);
        Serial.print("\t");
        Serial.println(aaWorld.z);
#endif

#ifdef OUTPUT_TEAPOT
        // display quaternion values in InvenSense Teapot demo format:
        teapotPacket[2] = fifoBuffer[0];
        teapotPacket[3] = fifoBuffer[1];
        teapotPacket[4] = fifoBuffer[4];
        teapotPacket[5] = fifoBuffer[5];
        teapotPacket[6] = fifoBuffer[8];
        teapotPacket[7] = fifoBuffer[9];
        teapotPacket[8] = fifoBuffer[12];
        teapotPacket[9] = fifoBuffer[13];
        Serial.write(teapotPacket, 14);
        teapotPacket[11]++; // packetCount, loops at 0xFF on purpose
#endif

        // blink LED to indicate activity
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);
    }
}


// ================================================================
// ===                      SHT15 Functions                     ===
// ================================================================

// commands for reading/sending data to a SHTx sensor
int shiftIn(int dataPin, int clockPin, int numBits) {
    int ret = 0;

    for (int i = 0; i < numBits; ++i) {
        digitalWrite(clockPin, HIGH);
        //delay(10); not needed :)
        ret = ret * 2 + digitalRead(dataPin);
        digitalWrite(clockPin, LOW);
    }
    return (ret);
}

// send a command to the SHTx sensor
void sendCommandSHT(int command, int dataPin, int clockPin) {
    int ack;

    // transmission start
    pinMode(dataPin, OUTPUT);
    pinMode(clockPin, OUTPUT);
    digitalWrite(dataPin, HIGH);
    digitalWrite(clockPin, HIGH);
    digitalWrite(dataPin, LOW);
    digitalWrite(clockPin, LOW);
    digitalWrite(clockPin, HIGH);
    digitalWrite(dataPin, HIGH);
    digitalWrite(clockPin, LOW);

    // shift out the command (the 3 MSB are address and must be 000, the last 5 bits are the command)
    shiftOut(dataPin, clockPin, MSBFIRST, command);

    // verify we get the right ACK
    digitalWrite(clockPin, HIGH);
    pinMode(dataPin, INPUT);
    ack = digitalRead(dataPin);
    if (ack != LOW)
        Serial.println("ACK error 0");
    digitalWrite(clockPin, LOW);
    ack = digitalRead(dataPin);
    if (ack != HIGH)
        Serial.println("ACK error 1");
}

// wait for the SHTx answer
void waitForResultSHT(int dataPin) {
    int ack;

    pinMode(dataPin, INPUT);
    for (int i = 0; i < 100; ++i) {
        delay(10);
        ack = digitalRead(dataPin);
        if (ack == LOW)
            break;
    }
    if (ack == HIGH)
        Serial.println("ACK error 2");
}

// get data from the SHTx sensor
int getData16SHT(int dataPin, int clockPin) {
    int val;

    // get the MSB (most significant bits)
    pinMode(dataPin, INPUT);
    pinMode(clockPin, OUTPUT);
    val = shiftIn(dataPin, clockPin, 8);
    val *= 256; // this is equivalent to val << 8;

    // send the required ACK
    pinMode(dataPin, OUTPUT);
    digitalWrite(dataPin, HIGH);
    digitalWrite(dataPin, LOW);
    digitalWrite(clockPin, HIGH);
    digitalWrite(clockPin, LOW);

    // get the LSB (less significant bits)
    pinMode(dataPin, INPUT);
    val |= shiftIn(dataPin, clockPin, 8);
    return val;
}

// skip CRC data from the SHTx sensor
void skipCrcSHT(int dataPin, int clockPin) {
    pinMode(dataPin, OUTPUT);
    pinMode(clockPin, OUTPUT);
    digitalWrite(dataPin, HIGH);
    digitalWrite(clockPin, HIGH);
    digitalWrite(clockPin, LOW);
}

// ================================================================
// ===                     BMP085 Functions                     ===
// ================================================================
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
// Stores all of the bmp085's calibration values into global variables
// Calibration values are required to calculate temp and pressure
// This function should be called at the beginning of the program
void bmp085Calibration() {
    ac1 = bmp085ReadInt(0xAA);
    ac2 = bmp085ReadInt(0xAC);
    ac3 = bmp085ReadInt(0xAE);
    ac4 = bmp085ReadInt(0xB0);
    ac5 = bmp085ReadInt(0xB2);
    ac6 = bmp085ReadInt(0xB4);
    b1 = bmp085ReadInt(0xB6);
    b2 = bmp085ReadInt(0xB8);
    mb = bmp085ReadInt(0xBA);
    mc = bmp085ReadInt(0xBC);
    md = bmp085ReadInt(0xBE);
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
long bmp085GetPressure(unsigned long up) {
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

    return p;
}

// Read 1 byte from the BMP085 at 'address'
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

// ================================================================
// ===                     MPU6050 Functions                    ===
// ================================================================

void dmpDataReady() {
    mpuInterrupt = true;
}

