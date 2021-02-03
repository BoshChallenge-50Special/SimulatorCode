# Traffic Sign Detection

## detection_only.py 
To run, need to download the video.mp4 and modify 'cv2.VideoCapture(<path_to_video>)'

The 2 thresholds for 'cv2.Canny(imgGray,threshold1, threshold2)' need to be tuned to detect signals and not other things. Once the detection only works well, extract traffic sign images to retrain the cnn model.  
'cnn_model_7.h5' was build with 'TrafficSign_Elisa.ipynb' on colab using the GTSRB dataset. 

## traffic_sign_detection.py
Need to rebuild model 'cnn_model_7.h5' to accept varying input 0 of layer sequential sizes or change inputs to fixed size.
