import cv2 
import numpy as np 
import time
import math
import serial
import sys 

from robot import VirtualArm, isValidPosition, getAngles, degreesToRadians, radiansToDegrees, getArmLengths

SERIAL_NAME = "COM4"
SERIAL_BAUD = 9600

close_grip = False

class Arduino:
	def __init__(self, name, baud=9600):
		self.name = name 
		self.baud = baud
		# uncomment later
		self.serial = serial.Serial(self.name, self.baud)
	def open(self):
		self.serial = serial.Serial(self.name, self.baud)
	def close(self):
		self.serial.close()
	def writeToStream(self, text):
		# uncomment late
		self.serial.write((text+'\n').encode())
		pass
class Reference:
	def __init__(self):
		self.patch = None
		self.points = []
		self.starting_point = None
	def add_point(self, point):
		self.points.append(point)

	def get_patch_color(self):
		while True:
			ret, frame = cam.read() 
			# add text to the screen that says, press q to lock the color
			frame = cv2.putText(frame, "Press r to lock-in the patch color", (200, 170), cv2.FONT_HERSHEY_SIMPLEX, .6, (0,0,255), 1, cv2.LINE_AA)
			cv2.rectangle(frame, (200, 200), (245,245), (0,0,255), 2)
			cv2.imshow("mouseRGB", frame)
			key = cv2.waitKey(10)
			if key == ord('r'):
				subframe = frame[202:243, 202:243]
				break 
		(h,w,d) = subframe.shape
		blues = 0
		reds = 0
		greens = 0
		for i in range(h):
			for j in range(w):
				blues+= subframe.item(i,j,0)
				greens+= subframe.item(i,j,1)
				reds+= subframe.item(i,j,2)

		blue_avg = blues/(h*w)
		green_avg = greens/(h*w)
		red_avg = reds/(h*w)
		self.patch = Patch((blue_avg, green_avg,red_avg))


class Patch:	
	def __init__(self, colors):
		self.colors = colors

		self.top = 100000
		self.bottom = 0
		self.left = 1000
		self.right = 0
	def reset(self):
		self.top = 100000
		self.bottom = 0
		self.left = 1000
		self.right = 0		
	def getCenter(self):
		return  ( int((self.left+self.right)/2), int((self.bottom + self.top)/2))




def mouseRGB(event,x,y,flags,param):
    if event == cv2.EVENT_LBUTTONDOWN: #checks mouse left button down condition
        colorsB = frame[y,x,0]
        colorsG = frame[y,x,1]
        colorsR = frame[y,x,2]
        colors = frame[y,x]
        print("Red: ",colorsR)
        print("Green: ",colorsG)
        print("Blue: ",colorsB)
        print("BRG Format: ",colors)
        print("Coordinates of pixel: X: ",x,"Y: ",y)
        

def findPatches(image,patches,r):
	(h, w, d) = image.shape 
	for p in patches:
		p.reset()
	for i in range(h):
		for j in range(w):    # blue, green,  red 
        	# 0-255
			blue = image.item(i,j,0)
			green = image.item(i,j,1) 
			red =  image.item(i,j,2)
			in_patch = False 
			for patch in patches:
				
				p = patch.colors

				if p[0] == -1 or (blue >= p[0] -r and blue <= p[0] + r):
					if p[1] == -1 or (green >= p[1] -r and green <= p[1] + r):
						if p[2] == -1 or (red >= p[2] - r and red <= p[2] + r):
							
							in_patch = True 

							if i < patch.top:
								patch.top = i
							if i > patch.bottom:
								patch.bottom = i 
							if j > patch.right:
								patch.right = j
							if j < patch.left: 
								patch.left =j


			if not in_patch:
				image.itemset((i,j,0), 0)
				image.itemset((i,j,1), 0)
				image.itemset((i,j,2), 0)



def track_references(refs):
	global close_grip
	points = []
	patches = []
	for r in refs:
		patches.append(r.patch)

	wrist = refs[0]
	while True: 
	    ret, frame = cam.read() 

	    frame = cv2.flip(frame,1)

	    frame2 = frame.copy()
	    findPatches(frame2, patches, 25)
	    frame2 = np.uint8(frame2)

	    wrist_point = patches[0].getCenter()
	    palm_point = patches[1].getCenter()

	    if palm_point[1] < 0 or palm_point[1] == 50000:
	    	print("no palm, so close grip")
	    	if not close_grip:
	    		arduino.writeToStream("1")
	    		close_grip = True
	    else:
	    	#print("open grip")
	    	if close_grip:
		    	arduino.writeToStream("0")
		    	close_grip = False

	    if wrist.starting_point is None:
	    	wrist.starting_point = wrist_point 
	    	# set the starting poitn....
	    	calibrateArm(frame.shape, wrist_point)
	    if wrist_point[1] < 0 or wrist_point[1] == 50000:
	    	# not a valid point..... could not find patch...
	    	# use the previous known point....
	    	if len(points) > 0:
	    		wrist_point = points[len(points)-1]
	    	else:
		    	wrist_point = (0,0)

	    points.append(wrist_point)
	    wrist.add_point(wrist_point)

	    if STARTING_POSITION[0] != 0:
		    addArmsToFrame(frame2, wrist_point)

	    cv2.imshow("cam", frame) 
	    cv2.imshow("mouseRGB", frame2)
	    #cv2.imshow("dsdf", frame3)
	    key = cv2.waitKey(10)
	    if key == ord('q'):
	    	#print(len(wrist.points))
	    	#print(wrist.points)
	    	break 


cam = cv2.VideoCapture(0)

STARTING_POSITION = (0,0)

def calibrateArm(shape, p):
	global STARTING_POSITION
	h = shape[0]
	w = shape[1]
	x = w - p[0] 	# 509
	y = h - p[1]	# 94
	STARTING_POSITION = (x,y)
	
def addArmsToFrame(frame, p):

	arm_lengths = getArmLengths()
	max_length = 0
	arm_count = len(arm_lengths)
	for i in range(arm_count):
		max_length += arm_lengths[i] 

	h = frame.shape[0]
	w = frame.shape[1]
	x,y = translatePoint(frame.shape, p, max_length)
	y = 50-y
	if not arm.isValidPosition(x,y) and x>0 and y>0:
		# choose a new x,y that is in the same line from the origin but closer to origin...
		# create a new (x,y)
		a1 = arm.segment_1
		a2 = arm.segment_2
		max_length = a1+a2
		slope = y/x 
		counter = 0
		while not arm.isValidPosition(x,y) and counter < 20:
			x-= .5 
			y-= .5*slope 
			counter+=1

	if arm.isValidPosition(x,y):
		angles = arm.getAngles(x,y)
		# send the angles to the arduino....
		arduino.writeToStream(str(angles[0]) + ";"+str(angles[1]))

		arm.drawArmOnFrame(frame, (0,480))

# purpose:
# 	translate a webcam position to a robotic arm position (smaller scale...)
def translatePoint(shape, p, max_length):


	h = shape[0]
	w = shape[1]
	x = (p[0] / h ) * max_length
	
	y = (p[1] / h ) * max_length 

	return (x, y) 



arm = VirtualArm(30, 20)
arduino = Arduino(SERIAL_NAME, SERIAL_BAUD)

def main_color_patch():
	# find the color values of our patches. 

	wrist = Reference()
	wrist.get_patch_color()

	palm = Reference()
	palm.get_patch_color()

	#hand = Reference()
	#hand.get_patch_color()
	references = []
	references.append(wrist)
	references.append(palm)
	time.sleep(2)
	track_references(references)
	# send arm angles to arduino

def main_bg_subtract():
	pass 

mode = None
if len(sys.argv) > 1:
	mode = sys.argv[1] 
if mode == 'bg_subtract':
	main_bg_subtract()
else:
	main_color_patch()

