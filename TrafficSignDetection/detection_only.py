import cv2
import numpy as np

print("Package imported")

frameWidth = 640
frameHeight = 480
#cap = cv2.VideoCapture(0) #for camera input
# mentioning absolute path of the video
video_path = "C:\\Users\\Elisa\\PycharmProjects\\TrafficSignDetection\\video.mp4"
cap = cv2.VideoCapture(video_path)
cap.set(3,frameWidth)
cap.set(4,frameHeight)

#from number of points to shape
def shape(i):
    shape = {0: 'other',
         1: 'other',
         2: 'line',
         3: 'triangle',
         4: 'rectangle',
         5: 'pentagone',
         6: 'hexagon',
         7: 'heptagon',
         8: 'octagon',
         9: 'nonagone',
         10: 'other',
         11: 'other',
         12: 'other',
         13: 'other'}
    return shape.get(i, "Invalid shape")

#function to call whenever there is a change in the value
def empty(a):
  pass

#creating the trackbars to control the parameters
cv2.namedWindow("Parameters")
cv2.resizeWindow("Parameters",640,240)
cv2.createTrackbar("Threshold1","Parameters",26,255,empty)
cv2.createTrackbar("Threshold2","Parameters",45,255,empty)
cv2.createTrackbar("AreaMin","Parameters",1500,30000,empty)
cv2.createTrackbar("AreaMax","Parameters",3000,30000,empty)


def getContours(img,imgContour):
    contours,hierarchy = cv2.findContours(img,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)
    for cnt in contours:
        area = cv2.contourArea(cnt)
        areaMin = cv2.getTrackbarPos("AreaMin","Parameters")
        areaMax = cv2.getTrackbarPos("AreaMax","Parameters")
        if (area > areaMin and area < areaMax):
            cv2.drawContours(imgContour,cnt, -1, (255,0,255),7)
            peri = cv2.arcLength(cnt,True)
            approx = cv2.approxPolyDP(cnt,0.02*peri,True)
            # print(len(approx))
            x ,y ,w ,h = cv2.boundingRect(approx)
            imgCropped = img[y:y + h, x:x + w]
            cv2.rectangle(imgContour,(x,y),(x+w,y+h),(0,255,0),5)
            cv2.putText(imgContour, "Prediction:" + shape(len(approx)),(x+w+20,y+20),cv2.FONT_HERSHEY_COMPLEX,0.7,(0,255,0),2)
            cv2.putText(imgContour, "Area:"+str(int(area)),(x+w+20,y+45),cv2.FONT_HERSHEY_COMPLEX,0.7,(0,255,0),2)

while(True):
    success,img =cap.read()
    imgContour = img.copy()
    y_pred = []

    imgBlur = cv2.GaussianBlur(img, (7,7),1)
    imgGray = cv2.cvtColor(imgBlur,cv2.COLOR_BGR2GRAY)

    threshold1 = cv2.getTrackbarPos("Threshold1","Parameters")
    threshold2 = cv2.getTrackbarPos("Threshold2","Parameters")
    #canny edge detector
    imgCanny = cv2.Canny(imgGray,threshold1, threshold2)
    #dilate image
    kernel = np.ones((5,5))
    imgDil = cv2.dilate(imgCanny,kernel,iterations = 1)
    getContours(imgDil,imgContour)
    cv2.imshow("Result",imgContour)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
