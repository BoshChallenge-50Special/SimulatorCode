import cv2
import numpy as np
from matplotlib import pyplot as plt
from os import listdir
# local modules
from common import clock, mosaic
import pickle
import joblib


from sklearn import svm

#Parameter
SIZE = 32
CLASS_NUMBER = 11


def load_traffic_dataset():
    dataset = []
    labels = []
    for sign_type in range(CLASS_NUMBER):
        print(sign_type)
        sign_list = listdir("/home/morettini18/dataset_bosch/{}".format(sign_type))
        for sign_file in sign_list:
            if '.png' in sign_file:
                path = "/home/morettini18/dataset_bosch/{}/{}".format(sign_type,sign_file)
                print(path)
                img = cv2.imread(path,0)
                img = cv2.resize(img, (SIZE, SIZE))
                img = np.reshape(img, [SIZE, SIZE])
                dataset.append(img)
                labels.append(sign_type)
    return np.array(dataset), np.array(labels)

def load_model(path):
    #model = SVM()
    #model.model = cv2.ml.SVM_load(path)
    loaded_model=SVM()
    loaded_model.load(path)
    return loaded_model

def deskew(img):
    m = cv2.moments(img)
    if abs(m['mu02']) < 1e-2:
        return img.copy()
    skew = m['mu11']/m['mu02']
    M = np.float32([[1, skew, -0.5*SIZE*skew], [0, 1, 0]])
    img = cv2.warpAffine(img, M, (SIZE, SIZE), flags=cv2.WARP_INVERSE_MAP | cv2.INTER_LINEAR)
    return img

class StatModel(object):
    def load(self, fn):
        #self.model.load(fn)  # Known bug: https://github.com/opencv/opencv/issues/4969
        # load
        print("Loading model from model.pkl")
        self.model = joblib.load(fn)

    def save(self, fn):
        #self.model.save(fn)
        # save
        print("Saving model to model.pkl")
        joblib.dump(self.model, fn)

class SVM(StatModel):
    def __init__(self, C = 12.5, gamma = 0.50625):
        #self.model = cv2.ml.SVM_create()
        #self.model.setGamma(gamma)
        #self.model.setC(C)
        #self.model.setKernel(cv2.ml.SVM_RBF)
        #self.model.setType(cv2.ml.SVM_C_SVC)
        kernel="rbf"
        probability=True
        verbose=False
        self.model = svm.SVC(kernel=kernel, probability=probability, gamma=gamma,verbose=verbose)

    def train(self, samples, responses):
        #self.model.train(samples, cv2.ml.ROW_SAMPLE, responses)
        self.model.fit(samples, responses)
        print(self.model)

    def predict(self, samples):
        #return self.model.predict(samples)[1].ravel()
        prediction=self.model.predict(samples)
        return prediction


def evaluate_model(model, data, samples, labels):
    resp = model.predict(samples)
    print(resp)
    err = (labels != resp).mean()
    print('Accuracy: %.2f %%' % ((1 - err)*100))

    confusion = np.zeros((10, 10), np.int32)
    for i, j in zip(labels, resp):
        confusion[int(i), int(j)] += 1
    print('confusion matrix:')
    print(confusion)

    vis = []
    for img, flag in zip(data, resp == labels):
        img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        if not flag:
            img[...,:2] = 0

        vis.append(img)
    return mosaic(16, vis)

def preprocess_simple(data):
    return np.float32(data).reshape(-1, SIZE*SIZE) / 255.0


def get_hog() :
    winSize = (20,20)
    blockSize = (10,10)
    blockStride = (5,5)
    cellSize = (10,10)
    nbins = 9
    derivAperture = 1
    winSigma = -1.
    histogramNormType = 0
    L2HysThreshold = 0.2
    gammaCorrection = 1
    nlevels = 64
    signedGradient = True

    hog = cv2.HOGDescriptor(winSize,blockSize,blockStride,cellSize,nbins,derivAperture,winSigma,histogramNormType,L2HysThreshold,gammaCorrection,nlevels, signedGradient)

    return hog
    affine_flags = cv2.WARP_INVERSE_MAP|cv2.INTER_LINEAR


def training():
    print('Loading data from data.png ... ')
    # Load data.
    #data, labels = load_data('data.png')
    data, labels = load_traffic_dataset()
    print(data.shape)
    print('Shuffle data ... ')
    # Shuffle data
    rand = np.random.RandomState(10)
    shuffle = rand.permutation(len(data))
    data, labels = data[shuffle], labels[shuffle]

    print('Deskew images ... ')
    data_deskewed = list(map(deskew, data))

    print('Defining HoG parameters ...')
    # HoG feature descriptor
    hog = get_hog()

    print('Calculating HoG descriptor for every image ... ')
    hog_descriptors = []
    for img in data_deskewed:
        hog_descriptors.append(hog.compute(img))
    hog_descriptors = np.squeeze(hog_descriptors)

    print('Spliting data into training (90%) and test set (10%)... ')
    train_n=int(0.9*len(hog_descriptors))
    data_train, data_test = np.split(data_deskewed, [train_n])
    hog_descriptors_train, hog_descriptors_test = np.split(hog_descriptors, [train_n])
    labels_train, labels_test = np.split(labels, [train_n])


    print('Training SVM model ...')
    model = SVM()
    model.train(hog_descriptors_train, labels_train)

    #print('Saving SVM model ...')
    #model.save('./src/startup_package/src/SimulatorCode/TrafficSignDetection/data_svm.dat')
    # save the model to disk
    #pickle.dump(model, open('finalized_model.sav', 'wb'))

    return model

def getLabel(model, data):
    gray = cv2.cvtColor(data, cv2.COLOR_BGR2GRAY)
    img = [cv2.resize(gray,(SIZE,SIZE))]
    #print(np.array(img).shape)
    img_deskewed = list(map(deskew, img))
    hog = get_hog()
    hog_descriptors = np.array([hog.compute(img_deskewed[0])])
    hog_descriptors = np.reshape(hog_descriptors, [-1, hog_descriptors.shape[1]])

    return int(model.predict(hog_descriptors)[0])