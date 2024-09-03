import cv2 as cv 


def rescaleFrame(frame, scale=0.2): #we are making a fuction for resizing the img/vid
    width = int(frame.shape[1] * scale)
    height = int(frame.shape[1] * scale)
    dimensions = (width,height)

#this fuc will will for images,vid,live ect

    return cv.resize(frame, dimensions, interpolation=cv.INTER_AREA)

def changeRes(width,height): # used for changing the resoultion of vid
    capture.set(10,width)
    capture.set(10,height)
# but this fuc will only work for live web cam vids


capture  = cv.VideoCapture(0)

while True:
    isTrue, frame = capture.read()

    frame_resized = rescaleFrame(frame)

    cv.imshow("Video",frame)
    cv.imshow("videoResized",frame_resized)

    if cv.waitKey(20) & 0xFF == ord('d'):
        break



capture.release()
cv.destroyAllWindows()