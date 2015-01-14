import serial
import time
import matplotlib.pyplot as plt  
import matplotlib.animation as anim  
from collections import deque
import random

serialPort = serial.Serial('/dev/cu.usbserial-A601SFE1', 9600)
start = time.time()
#accelX|accelY|accelZ|magX|magY|magZ|gyroX|gyroY|gyroZ|pressure|temperature|altitude


MAX_X = 100   #width of graph
MAX_Y = 60  #height of graph
 
# intialize line to horizontal line on 0
line = deque([0.0]*MAX_X, maxlen=MAX_X)


 
def update(fn, l2d):
	data_to_parse = serialPort.readline();
	data_to_parse = data_to_parse.strip().split("|")
	if len(data_to_parse) == 12:
		
		temp  = data_to_parse[10]
		currentT_ = time.time()
		delta = currentT_- start

		line.append(temp)
		l2d.set_data(range(-MAX_X/2, MAX_X/2), line)

fig = plt.figure()

a = plt.axes(xlim=(0,MAX_X/2), ylim=(0,MAX_Y))

l1, = a.plot([], [])
ani = anim.FuncAnimation(fig, update, fargs=(l1,), interval=50)
 
plt.show()
