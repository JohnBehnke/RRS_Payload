
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
