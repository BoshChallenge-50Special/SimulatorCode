import cv2
import numpy as np
from keras.models import Model, load_model

print("Package imported")

frameWidth = 640
frameHeight = 480

# Input: set cv2.VideoCapture(X) X = 0 for camera input or X = absolute path of the video
# for file video input
#cap = cv2.VideoCapture(0) #for camera input
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

#mapping to sign names for classification algorithm
def signals(i):
    signal = {0: 'Speed limit (20km/h)',
         1: 'Speed limit (30km/h)',
         2: 'Speed limit (50km/h)',
         3: 'Speed limit (60km/h)',
         4: 'Speed limit (70km/h)',
         5: 'Speed limit (80km/h)',
         6: 'End of Speed limit (80km/h)',
         7: 'Speed limit (100km/h)',
         8: 'Speed limit (120km/h)',
         9: 'No passing',
         10: 'No passing for vehicles over 3.5 metric tons',
         11: 'Right-of-way at the next intersection',
         12: 'Priority road',
         13: 'Yield',
         14: 'Stop',
         15: 'No vehicles',
         16: 'Vehicles over 3.5 metric tons prohibited',
         17: 'No entry',
         18: 'General caution',
         19: 'Dangerous curve to the left',
         20: 'Dangerous curve to the right',
         21: 'Double curve',
         22: 'Bumpy road',
         23: 'Slippery road',
         24: 'Road narrows on the right',
         25: 'Road work',
         26: 'Traffic signals',
         27: 'Pedestrians',
         28: 'Children crossing',
         29: 'Bicycles crossing',
         30: 'Beware of ice / snow',
         31: 'Wild animals crossing',
         32: 'End of all speed and passing limits',
         33: 'Turn right ahead',
         34: 'Turn left ahead',
         35: 'Ahead only',
         36: 'Go straight or right',
         37: 'Go straight or left',
         38: 'Keep right',
         39: 'Keep left',
         40: 'Roundabout mandatory',
         41: 'End of no passing',
         42: 'End of no passing by vehicles over 3.5 metric tons',
         43: 'Parking',
         44: 'Highway entrance sign',
         45: 'Highway exit sign'
    }
    return signal.get(i, "Invalid signal")

#function to call whenever there is a change in the value
def empty(a):
  pass

#creating the trackbars to control the parameters
cv2.namedWindow("Parameters")
cv2.resizeWindow("Parameters",640,240)
cv2.createTrackbar("Threshold1","Parameters",23,255,empty)
cv2.createTrackbar("Threshold2","Parameters",20,255,empty)
cv2.createTrackbar("Area","Parameters",5000,30000,empty)

#importing cnn model for traffic sign recognition
filename = 'cnn_model_' + '7' + '.h5'
cnn_model = load_model(filename)

def getContours(img,imgContour):
    contours,hierarchy = cv2.findContours(img,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)
    for cnt in contours:
        area = cv2.contourArea(cnt)
        areaMin = cv2.getTrackbarPos("Area","Parameters")
        if area > areaMin:
            cv2.drawContours(imgContour,cnt, -1, (255,0,255),7)
            peri = cv2.arcLength(cnt,True)
            approx = cv2.approxPolyDP(cnt,0.02*peri,True)
            # print(len(approx))
            x ,y ,w ,h = cv2.boundingRect(approx)
            imgCropped = img[y:y + h, x:x + w]
            y_pred = cnn_model.predict(imgCropped)
            cv2.rectangle(imgContour,(x,y),(x+w,y+h),(0,255,0),5)
            cv2.putText(imgContour, "Prediction:" + signals(y_pred),(x+w+20,y+20),cv2.FONT_HERSHEY_COMPLEX,0.7,(0,255,0),2)
            cv2.putText(imgContour, "Area:"+str(int(area)),(x+w+20,y+45),cv2.FONT_HERSHEY_COMPLEX,0.7,(0,255,0),2)

while True:
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
