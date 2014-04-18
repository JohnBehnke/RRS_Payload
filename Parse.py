import serial
import pygmaps 
import time



GPS = open('gps.txt', 'w')
ACC = open('acc.txt', 'w')
TEMP = open('temp.txt', 'w')
HUM = open('hum.txt', 'w')
PRES = open('pres.txt', 'w')

mymap = pygmaps.maps(42.729358, -73.674453, 16)

ser = serial.Serial('/dev/cu.usbserial-AD02ICC0', 57600)


while True:


	data_to_parse = ser.readline();

	data_to_parse = data_to_parse.strip().split("|")

	A_Data = data_to_parse[:3]

	T_Data = data_to_parse[3]

	P_Data = data_to_parse[4]

	H_Data = data_to_parse[5]

	G_Log = data_to_parse[-2]

	G_Lat = data_to_parse[-1]


	try:
		
		GPS.seek(0, 2)

		GPS.write(G_Log+"|"+G_Lat+"\n")

		ACC.seek(0, 2)

		ACC.write(A_Data[0]+"|"+A_Data[1]+"|"+A_Data[2]+"\n")

		TEMP.seek(0,2)

		TEMP.write(T_Data+"\n")

		HUM.seek(0,2)

		HUM.write(H_Data+"\n")

		PRES.seek(0,2)

		PRES.write(P_Data+"\n")

		
		
		GPS_INTS=map(float,data_to_parse[-2:])

		
		mymap.addpoint(GPS_INTS[0], GPS_INTS[1],"#0000FF")

		mymap.addradpoint(GPS_INTS[0], GPS_INTS[1], 10, "#FF0000")
		mymap.draw('~/Desktop/GPS.html')
	except ValueError:
		pass



	print data_to_parse