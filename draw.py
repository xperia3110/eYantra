import cv2 as cv 
import numpy as np 

blank = np.zeros((500,500,3) , dtype = 'uint8')

# zeroes(width,height,no of color channels ) --- function
#we can draw in an image or use a blank image to draw using above fun
# uint8 is a data type for an image

cv.imshow("blank",blank)

#painting the image

blank[:] = 0,255,0 #color code for green
cv.imshow('Green',blank)

#to color a certain parts of the image we use :

blank[200:300, 300:400] = 0,0,225 #blank acepts values in pixels
cv.imshow("part",blank)

#drawing a rectangle in the image

cv.rectangle(blank, (0,0), (250,250), (0,198,100), thickness=2)
# values taken = image for drwaing, (0,0) means orgin, end point, color, thickness
cv.imshow("rectangle",blank)

#to draw a circle use cv.circle and pass similar vlaues

#to draw line we us ecv.line

#add text
cv.putText(blank,"this is collins",(255,255),cv.FONT_HERSHEY_TRIPLEX, 1.0,(0,0,0),thickness=5)
cv.imshow("text",blank)

cv.waitKey(0)