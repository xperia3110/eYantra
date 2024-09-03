import cv2 as cv 


#imread is used to read the image from the system


img = cv.imread('/Users/collinsshibi/Desktop/vs_code/python/open-cv/imgs/36ea0d73-c165-4a38-8940-70d6b9aab942.jpg')


#imshow is used to view the image that is been read

cv.imshow("Mini collins",img)


#waitKey is used to wait/delay the image for pressing a button

cv.waitKey(0) #0 means no wait instant