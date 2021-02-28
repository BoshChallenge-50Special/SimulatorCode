import cv2
#while success:
fname = '/home/ebllaei/Downloads/dataset/list.txt'
image_handler = open(fname, 'r')
for line in image_handler:
    '''
    victim - the image I actually do the processing on
    watch - this is where I draw the rectangles and whatnot so it can be tested in practice
    centers - the centers of the regions of interes
    '''
    img = cv2.imread('/home/ebllaei/Downloads/dataset/img_1000.png')
    cv2.imshow("img",img)
    # A dirty drick, unsure if still necessary, but I will leave it here.
    print(line)
    watch  = cv2.imread(str(line))
    print(watch)
    cv2.imshow("watch",watch)