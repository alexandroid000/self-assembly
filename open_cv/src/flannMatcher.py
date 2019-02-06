'''
Call with 'python flannMatcher.py objectImage.png sceneImage.png'
Where objectImage.png and sceneImage.png are in the SamplePictures folder

Current to-dos:
- Use general minimum/maximum distance from 4B-ML-1 to establish guidelines for connected shapes. I.e. if it hits close to min, they are definitely connected.

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
                        # print("Location:")
                        # print(kp1[m.queryIdx].pt)
                        # print(kp2[m.trainIdx].pt)
        draw_params = dict(matchColor = (0,255,0),
                        singlePointColor = (255,0,0),
                        matchesMask = matchesMask,
                        flags = cv.DrawMatchesFlags_DEFAULT)
        img3 = cv.drawMatchesKnn(img1,kp1,img2,kp2,matches,None,**draw_params)

        # Part 2: Homography
        MIN_MATCH_COUNT = 10       
       
        # store all the good matches as per Lowe's ratio test.
        good = []
        for m,n in matches:
                if m.distance < 0.9*n.distance:
                        good.append(m)
        if len(good)>MIN_MATCH_COUNT:
                src_pts = np.float32([ kp1[m.queryIdx].pt for m in good ]).reshape(-1,1,2)
                dst_pts = np.float32([ kp2[m.trainIdx].pt for m in good ]).reshape(-1,1,2)
                M, mask = cv.findHomography(src_pts, dst_pts, cv.RANSAC,5.0)
                bperror = 0
                for i in range(len(good)):
                        h11 = M[0][0]
                        h12 = M[0][1]
                        h13 = M[0][2]
                        h21 = M[1][0]
                        h22 = M[1][1]
                        h23 = M[1][2]
                        h31 = M[2][0]
                        h32 = M[2][1]
                        h33 = M[2][2]
                        xi = src_pts[i][0][0]
                        yi = src_pts[i][0][1]
                        xpi = dst_pts[i][0][0]
                        ypi = dst_pts[i][0][1]

                        bperror += (xpi - (h11*xi + h12*yi + h13)/(h31*xi + h32*yi + h33))**2 + (ypi - (h21*xi + h22*yi + h23)/(h31*xi + h32*yi + h33))**2
                print("BP Error: "+str(bperror))
        else:
                print( "Not enough matches are found - {}/{}".format(len(good), MIN_MATCH_COUNT) )

        plt.imshow(img3,),plt.show()


if __name__ == '__main__':
    main()