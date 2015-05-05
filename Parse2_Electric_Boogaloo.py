import serial
from Tkinter import *
import  ImageTk
from PIL import Image

import math as m

from mpl_toolkits.basemap import Basemap
import numpy as np
import matplotlib.pyplot as plt
width = 280000; lon_0 = -73.715328; lat_0 = 42.746616
#MAGIC NUMBERS


topLeftX =  42.746616
topLeftY = -73.715328

bottomRightX = 42.725936
bottomRightY = -73.668990 

xOffset = bottomRightX - topLeftX
yOffset = m.fabs(bottomRightY - topLeftY)
imageName = 'map.png'

radius = 10

#serialPort = serial.Serial('/dev/cu.usbserial-A601SFE1', 9600)


#42.728534, -73.682266

CX = 40.5374384

CY = 14.4701319

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
	m = Basemap(width=width,height=width,resolution='h',projection='aeqd',
            lat_0=lat_0,lon_0=lon_0)
	m.bluemarble(ax=None, scale=None)
	m.warpimage(image='bluemarble', scale=None)
	#m.arcgisimage(server='http://server.arcgisonline.com/ArcGIS', service='ESRI_Imagery_World_2D', xpixels=400, ypixels=None, dpi=96, verbose=False)
# fill background.
	m.drawmapboundary(fill_color='aqua')
	# draw coasts and fill continents.
	m.drawcoastlines(linewidth=0.5)
	#m.fillcontinents(color='coral',lake_color='aqua')
	# 20 degree graticule.
	m.drawparallels(np.arange(-80,81,20))
	m.drawmeridians(np.arange(-180,180,20))
	# draw a black dot at the center.
	xpt, ypt = m(lon_0, lat_0)
	m.plot([xpt],[ypt],'ko')
	# draw the title.
	plt.title('Azimuthal Equidistant Projection')
	plt.show()

	

	# for x in range(len(fakeDataLat)):
	# 	plotGPSMap(fakeDataLong[x],fakeDataLat[x],canvas)

	
	#canvas.update()

	