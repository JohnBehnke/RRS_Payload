
float pressure = bmp085GetPressure(bmp085ReadUP());
gps.f_get_position(&flat, &flon, &age);
float temperature = sht1x.readTemperatureC();
float humidity = sht1x.readHumidity();

Serial.print("OK|");
//Serial.print("|");
Serial.print(temperature);
//  Serial.println(" deg C");


//smartdelay(100);//just here to slow down the serial output - Easier to read
}
