import cv2 as cv 

#VideoCapture is used to capture a video

capture  = cv.VideoCapture(0)

#we can give int values to capture like 0,1 or 2
# or we can give path for a video
#if we use 0,1,2 it will use webcam of our system

#to read a video we will use while loop and read it frame by frame

while True:
    isTrue, frame = capture.read()

    # isTrue is a bool value to make sure that the video has been read sucessfully or not

    cv.imshow("Video",frame)

    #and to stop the video from playing infintley we use 

    if cv.waitKey(20) & 0xFF == ord('d'):
        break

# above code means that if we press d the video loop will stop

capture.release()
cv.destroyAllWindows() #used to close all windows

