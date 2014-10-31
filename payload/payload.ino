






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
