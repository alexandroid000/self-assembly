
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
    brightname = "../SamplePictures/Bright_" + str(sys.argv[1])
    blurname = "../SamplePictures/Blur_" + str(sys.argv[1])
    edgename = "../SamplePictures/Edges_" + str(sys.argv[1])
    
    binaryname = "../SamplePictures/Binary_" + str(sys.argv[1])
    binaryadaptivename = "../SamplePictures/BinaryAdaptive_" + str(sys.argv[1])
    
    origImage = cv.imread(origname)
    grayImage = cv.cvtColor(origImage,cv.COLOR_BGR2GRAY)
    

    alpha = 0.8 # Higher = more contrast
    gamma = 0 # Higher = brighter (little-to-no impact)
    blurNeighbors = 7 # Higher = larger neighborhood
    sigmaColor = 75 # Higher = color influence range (little-to-no impact)
    sigmaSpace = 75 # Higher = distance influence range
    binarythresh = 120 # Higher = darker binary scene

    brightImage = cv.addWeighted(grayImage, alpha, grayImage, 0, gamma)
    cv.imwrite(brightname, brightImage)

    blurImage = cv.bilateralFilter(brightImage, blurNeighbors, sigmaColor, sigmaSpace)#(d, sc, ss)
    cv.imwrite(blurname, blurImage)
    # d = diameter of pixel neighborhood, default = 9
    # sc = Sigma color, larger means more distant colors in neighborhood are mixed, default = 75
    # ss = Sigma space, larger means farther pixels with close colors will influence each other, default = 75
    
    edgeImage = cv.Canny(blurImage, 0, 80)#(l,u)
    # cv.imwrite(edgename, edgeImage)
    # u = upper threshold, pixel accepted as edge
    # l = lower threshold, pixel rejected as edge (between is accepted if next to accepted pixel)
    
    retval, binaryImage = cv.threshold(blurImage, binarythresh, 255, cv.THRESH_BINARY)
    cv.imwrite(binaryname, binaryImage)
    

   
    alpha = 0.8 # Higher = more contrast
    gamma = 0 # Higher = brighter (little-to-no impact)
    blurNeighbors = 7 # Higher = larger neighborhood
    sigmaColor = 75 # Higher = color influence range (little-to-no impact)
    sigmaSpace = 75 # Higher = distance influence range
    binarythresh = 120 # Higher = darker binary scene

    brightgrayImage = cv.addWeighted(grayImage, alpha, grayImage, 0, gamma)
    blurgrayImage = cv.bilateralFilter(brightgrayImage, 7, 200, 10)
    edgegrayImage = cv.Canny(blurgrayImage, 0, 80)
    cv.imwrite(edgename, edgegrayImage)

    pixelneighborhood = 51
    
    adaptivebinaryImage = cv.adaptiveThreshold(blurgrayImage,255,cv.ADAPTIVE_THRESH_GAUSSIAN_C,\
            cv.THRESH_BINARY,pixelneighborhood,4)
    cv.imwrite(binaryadaptivename, adaptivebinaryImage)



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

if __name__ == '__main__':
    main()