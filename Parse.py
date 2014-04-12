import serial
import pygmaps 

mymap = pygmaps.maps(42.729358, -73.674453, 16)
mymap.setgrids(37.42, 37.43, 0.001, -122.15, -122.14, 0.001)


ser = serial.Serial('/dev/tty.usbmodem1421', 57600)

while True:
	#print ser.readline()

	temp = ser.readline();

	temp  = temp.strip().split('|')

	try:
		x = temp[-2:]
		x=map(float,x)
		print x
		
		mymap.addpoint(x[0], x[1],"#0000FF")
		mymap.draw('./test.html')
	except ValueError:
		pass



	print temp