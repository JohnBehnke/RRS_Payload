# import Image and the graphics package 
from Tkinter import *
import  ImageTk

from PIL import Image

# open a SPIDER image and convert to byte

#im = Image.open('2.png')

root = Tk()  
# A root window for displaying objects

topLeftX =  42.746616
topLeftY = -73.715328

bottomRightX = 42.725936
bottomRightY = -73.668990 


 # Convert the Image object into a TkPhoto 

#tkimage = ImageTk.PhotoImage(im)

#Label(root, image=tkimage).pack() 
# Put it in the display window

#root.mainloop() # Start the GUI

#root = Tk()

im = Image.open('map.png')
im = im.resize((1732,1066),Image.ANTIALIAS)
canvas = Canvas(root, width = 1732, height = 1066)

canvas.grid(row = 0, column = 0)

Tkimage = ImageTk.PhotoImage(im)

canvas.create_image(0,0, image=Tkimage, anchor = NW)

root.mainloop()

