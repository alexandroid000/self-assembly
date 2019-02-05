
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
    bimage = "../SamplePictures/Blur_" + str(sys.argv[1])
    iname = "../SamplePictures/Edges_" + str(sys.argv[1])
    

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


if __name__ == '__main__':
    main()