import cv2
import matplotlib.pyplot as plt
#%matplotlib inline
import time
# read images
#img1 = cv2.imread('/home/morettini18/dataset_bosch/1/135.jpg')
#img2 = cv2.imread('/home/morettini18/dataset_bosch/1/33_ERROR.png')
#img3 = cv2.imread('/home/morettini18/dataset_bosch/4/83_ERROR.png')
#img4 = cv2.imread('/home/morettini18/dataset_bosch/4/83_ERROR.png')
#img5 = cv2.imread('/home/morettini18/dataset_bosch/4/83_ERROR.png')
#img6 = cv2.imread('/home/morettini18/dataset_bosch/4/83_ERROR.png')
#img7 = cv2.imread('/home/morettini18/dataset_bosch/4/83_ERROR.png')
#img8 = cv2.imread('/home/morettini18/dataset_bosch/4/83_ERROR.png')
#img6 = cv2.imread('/home/morettini18/dataset_bosch/4/83_ERROR.png')
#img7 = cv2.imread('/home/morettini18/dataset_bosch/4/83_ERROR.png')
#img8 = cv2.imread('/home/morettini18/dataset_bosch/4/83_ERROR.png')

#feature matching
bf = cv2.BFMatcher(cv2.NORM_L1, crossCheck=True)
sift = cv2.xfeatures2d.SIFT_create()

#img2 = cv2.imread('eiffel_1.jpg')
def get_label_sift(img):
    images=[]
    path="/home/morettini18/dataset_bosch/"
    images.append(cv2.imread(path + '0/0_ERROR.png'))
    images.append(cv2.imread(path + '1/1450_ERROR.png'))
    images.append(cv2.imread(path + '2/3020_ERROR.png'))
    images.append(cv2.imread(path + '3/00007.ppm'))
    images.append(cv2.imread(path + '4/2736_ERROR.png'))
    images.append(cv2.imread(path + '10/59.png'))
    images.append(cv2.imread(path + '6/2240_ERROR.png'))
    images.append(cv2.imread(path + '7/2553_ERROR.png'))
    images.append(cv2.imread(path + '8/3202_ERROR.png'))
    images.append(cv2.imread(path + '9/77_ERROR.png'))

    test_image = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    keypoints_1, descriptors_1 = sift.detectAndCompute(test_image,None)

    max=0
    best=0
    for i, im in enumerate(images):
        img2 = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)

        #sift

        keypoints_2, descriptors_2 = sift.detectAndCompute(img2,None)

        matches = bf.match(descriptors_1,descriptors_2)
        matches = sorted(matches, key = lambda x:x.distance)
        #print(str(i) + "   " + str(len(matches)))
        #result.append(len(mathces))
        #img3 = cv2.drawMatches(test_image, keypoints_1, img2, keypoints_2, matches[:50], img2, flags=2)
        #plt.imshow(img3),plt.show()
        #time.sleep(4)
        if(len(matches)>max):
            max=len(matches)
            best=i
    return best
