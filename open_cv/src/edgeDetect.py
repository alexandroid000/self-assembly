
import numpy as np
import cv2 as cv
import sys
import matplotlib.pyplot as plt

# Part 3: Edge Detection
def main():
    
    if len(sys.argv) != 2:
        print("Needs an image (1 argument)")
        return
            
    
    origname = "../SamplePictures/" + str(sys.argv[1])
    brightimage = "../SamplePictures/Bright_" + str(sys.argv[1])
    bimage = "../SamplePictures/Blur_" + str(sys.argv[1])
    iname = "../SamplePictures/Edges_" + str(sys.argv[1])
    binaryimage = "../SamplePictures/Binary_" + str(sys.argv[1])
    
    rawImage = cv.imread(origname)

    alpha = 1
    gamma = 0
    rawImage = cv.addWeighted(rawImage, alpha, rawImage, 0, gamma)

    bilateral_filtered_image = cv.bilateralFilter(rawImage, 7, 200, 20)#(d, sc, ss)
    # d = diameter of pixel neighborhood, default = 9
    # sc = Sigma color, larger means more distant colors in neighborhood are mixed, default = 75
    # ss = Sigma space, larger means farther pixels with close colors will influence each other, default = 75
    cv.imwrite(bimage, bilateral_filtered_image)
    edge_detected_image = cv.Canny(bilateral_filtered_image, 0, 80)#(l,u)
    # u = upper threshold, pixel accepted as edge
    # l = lower threshold, pixel rejected as edge (between is accepted if next to accepted pixel)
    
    cv.imwrite(iname,edge_detected_image)

    binimage = cv.threshold(origimage)
ret,thresh1 = cv.threshold(img,127,255,cv.THRESH_BINARY)
ret,thresh2 = cv.threshold(img,127,255,cv.THRESH_BINARY_INV)
ret,thresh3 = cv.threshold(img,127,255,cv.THRESH_TRUNC)
ret,thresh4 = cv.threshold(img,127,255,cv.THRESH_TOZERO)
ret,thresh5 = cv.threshold(img,127,255,cv.THRESH_TOZERO_INV)
titles = ['Original Image','BINARY','BINARY_INV','TRUNC','TOZERO','TOZERO_INV']
images = [img, thresh1, thresh2, thresh3, thresh4, thresh5]
for i in xrange(6):
    plt.subplot(2,3,i+1),plt.imshow(images[i],'gray')
    plt.title(titles[i])
    plt.xticks([]),plt.yticks([])
plt.show()

#     img = cv2.imread('shapes.png')
#     gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

#     ret,thresh = cv2.threshold(gray,127,255,1)

#     contours,h = cv2.findContours(thresh,1,2)

#     for cnt in contours:
#         approx = cv2.approxPolyDP(cnt,0.01*cv2.arcLength(cnt,True),True)
#         print len(approx)
#     if len(approx)==5:
#         print "pentagon"
#         cv2.drawContours(img,[cnt],0,255,-1)
#     elif len(approx)==3:
#         print "triangle"
#         cv2.drawContours(img,[cnt],0,(0,255,0),-1)
#     elif len(approx)==4:
#         print "square"
#         cv2.drawContours(img,[cnt],0,(0,0,255),-1)
#     elif len(approx) == 9:
#         print "half-circle"
#         cv2.drawContours(img,[cnt],0,(255,255,0),-1)
#     elif len(approx) > 15:
#         print "circle"
#         cv2.drawContours(img,[cnt],0,(0,255,255),-1)

# cv2.imshow('img',img)
# cv2.waitKey(0)
# cv2.destroyAllWindows()

if __name__ == '__main__':
    main()