import serial



#serialPort = serial.Serial('/dev/cu.usbserial-A601SFE1', 9600)





#accelX|accelY|accelZ|magX|magY|magZ|gyroX|gyroY|gyroZ|pressure|temperature|altitude|time


def tempReader(textFile):



	temp = []
	for line in open(textFile):
		info = line.strip().split('|')

		temp.append(info)


	return temp





# def aquireData(serialData):

# 	data_to_parse = serialData.readline().strip().split("|") 





if __name__ == "__main__":

	
	
	temp = tempReader("sample.txt")

	accelX = []
	accelY = []
	accelZ = []

	magX = []
	magY = []
	magZ = []

	gyroX = []
	gyroY = []
	gyroZ = []

	pressure = []
	temperature = []
	altitude = []

	timeDelta = []

	print temp

	