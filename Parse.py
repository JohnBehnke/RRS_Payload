import serial
import pygmaps 



# Data parser made by John Behnke for the Rensselaer Rocket Society
# This handels a serial stream containing GPS longitude and latitude
# Along with other sensor data sent over a serial stream and parses it







# Make a bunch of text files for each statistic
GPS = open('gps.txt', 'w')
ACC = open('acc.txt', 'w')
TEMP = open('temp.txt', 'w')
HUM = open('hum.txt', 'w')
PRES = open('pres.txt', 'w')
ALT = open('alt.txt', 'w')


# Make a new map using dummy coordinates. Ideally they are fairly close to your actual location
mymap = pygmaps.maps(42.729358, -73.674453, 16)

# Open a serial stream on the USB port. The string will change depending on
#Computer and USB port in use,
ser = serial.Serial('/dev/cu.usbserial-A601SFE1', 57600)


while True:

	# Take in the serial stream into a string called data_to_parse
	data_to_parse = ser.readline();

	#Split and strip into a list
	data_to_parse = data_to_parse.strip().split("|")

	#Make sure that the list has all the proper data in the right order
	if data_to_parse[0] == 'OK' and len(data_to_parse) == 11 and data_to_parse[-1] == 'OK':

		# Get rid of the 'OK' tags
		data_to_parse.pop(0)

		data_to_parse.pop(-1)

		#Store the data into appropriate strings
		A_Data = data_to_parse[:3]

		T_Data = data_to_parse[3]

		P_Data = data_to_parse[4]

		H_Data = data_to_parse[5]

		G_Log = data_to_parse[-4]

		G_Lat = data_to_parse[-3]

		ALt_Data = data_to_parse[-2]

		Time = data_to_parse[-1]
			

		#Start writing to the files
		GPS.seek(0, 2)

		GPS.write(Time + "|" + G_Log + "|" + G_Lat + "\n")

		ACC.seek(0, 2)

		ACC.write(Time + "|" + A_Data[0]+"|"+A_Data[1]+"|"+A_Data[2]+"\n")

		TEMP.seek(0,2)

		TEMP.write(Time + "|" + T_Data+"\n")

		HUM.seek(0,2)

		HUM.write(Time + "|" + H_Data+"\n")

		PRES.seek(0,2)

		PRES.write(Time + "|" + P_Data+"\n")

		ALT.seek(0,2)

		ALT.write(Time + "|" + ALt_Data+"\n")

		try:
			#Convert the strings for the long and lat into ints
			GPS_INTS=map(float,data_to_parse[-2:])

			#Add the point to the map and draw an uncertainty ring around it
			mymap.addpoint(GPS_INTS[0], GPS_INTS[1],"#0000FF")

			mymap.addradpoint(GPS_INTS[0], GPS_INTS[1], 10, "#FF0000")
			path = [(GPS_INTS[0],GPS_INTS[1]),(37.428, -122.145),(37.427, -122.145),(37.427, -122.146),(37.427, -122.146)]
			mymap.addpath(path,"#00FF00")

			#draw it to the file
			mymap.draw('./mymap.HTML')

			print data_to_parse
		except ValueError:
			pass
		

	else:
		pass



	