'''
Call with 'python flannMatcher.py objectImage.png sceneImage.png'
Where objectImage.png and sceneImage.png are in the SamplePictures folder

Current to-dos:
- With ball location, calculate distance to other balls
- Use general minimum/maximum distance from 4B-ML-1 to establish guidelines for connected shapes. I.e. if it hits close to min, they are definitely connected.
- For connected shapes, create fully-connected graph with distinct vertices and thick lines.
- Use homography transform on all templates with that number of connected shapes. Calculate back-projection error for each shape and compare.
- Use best homography to determine CoM and orientation


Problems:
- Determining between templates - edge detection?
        - How to score match
- Find way to encode ball positions such that a template can be made and compared with scene
        w/o similar keypoints
- Rigidbod
- Metric for keypoints and radius of descriptors
- Doesn't work well with generic image
- Ideally, all units the same color
'''

import numpy as np
import cv2 as cv
import sys
import matplotlib.pyplot as plt

def main():
        if len(sys.argv) != 3:
                print("Needs a query image and a scene image (2 arguments)")
                return

        # Get query image
        qimage = "../SamplePictures/" + str(sys.argv[1])
        img1 = cv.imread(qimage,cv.IMREAD_GRAYSCALE) 

        # Get scene to scan   
        simage = "../SamplePictures/" + str(sys.argv[2])    
        img2 = cv.imread(simage,cv.IMREAD_GRAYSCALE) 

        # Initiate SIFT detector
        sift = cv.xfeatures2d.SIFT_create()

        # find the keypoints and descriptors with SIFT
        kp1, des1 = sift.detectAndCompute(img1,None)
        kp2, des2 = sift.detectAndCompute(img2,None)

        # FLANN parameters
        FLANN_INDEX_KDTREE = 1
        index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
        search_params = dict(checks=50)   # or pass empty dictionary
        flann = cv.FlannBasedMatcher(index_params,search_params)
        matches = flann.knnMatch(des1,des2,k=2) # One of a few methods to match descriptors from an image pairs

        # Need to draw only good matches, so create a mask
        matchesMask = [[0,0] for i in range(len(matches))]

        # ratio test as per Lowe's paper
        for i, (m,n) in enumerate(matches):
                if m.distance < 0.9*n.distance:#Default: 0.7, lower = more selective
                        matchesMask[i]=[1,0]
                        print("Location:")
                        print(kp1[m.queryIdx].pt)
                        print(kp2[m.trainIdx].pt)
        draw_params = dict(matchColor = (0,255,0),
                        singlePointColor = (255,0,0),
                        matchesMask = matchesMask,
                        flags = cv.DrawMatchesFlags_DEFAULT)
        img3 = cv.drawMatchesKnn(img1,kp1,img2,kp2,matches,None,**draw_params)
        plt.imshow(img3,),plt.show()

        # Part 2: Homography


if __name__ == '__main__':
    main()