#!/usr/bin/env python

import numpy as np 
import cv2
import argparse
from collections import deque
import sys


def find_background(in_vid):
    cap = cv2.VideoCapture(in_vid)
    capSize = (int(cap.get(3)), int(cap.get(4)))    
    j=0
    video_frames = np.zeros((capSize[1], capSize[0], 3, 100))
    while(cap.isOpened()):
        ret, frame = cap.read()
        if ret == True:
            if j < 150:
                j+=1
                continue
            elif j < 250:
                video_frames[:,:,:,j-150] = frame
                j+=1
            else:
                break
        else:
            break
    cap.release()
    frame_bg = np.median(video_frames,3)
    return frame_bg


def track(in_vid):
    cap = cv2.VideoCapture(in_vid)
    fps = cap.get(cv2.CAP_PROP_FPS)
    print (fps)
    capSize = (int(cap.get(3)), int(cap.get(4)))
    min_ball_size = 10
    max_ball_size = 20

    background = find_background(in_vid)
    if True:
        fourcc = cv2.VideoWriter_fourcc(*'H264')
        vout = cv2.VideoWriter()
        filename = 'tracking.mp4'
        success = vout.open(filename,fourcc,fps,capSize,True)

    f = open("trajectory_data.txt", 'w')

    num_ball = 0
    accum_r = 0.0
    r_count = 0.0
    bd = 0 #balls detected?
    while bd < 15:
        success, frame = cap.read()

        if success: #have a frame
            ff = np.uint8((cv2.GaussianBlur(abs(frame-background), (3,3), 2)))
            ff = cv2.cvtColor(ff, cv2.COLOR_BGR2GRAY) #HoughCircles requires greyscale image
            circles = cv2.HoughCircles(ff, cv2.HOUGH_GRADIENT, 2, 17,param1=80,param2=25, minRadius = min_ball_size, maxRadius = max_ball_size)
            if circles is not None:
                circles = np.round(circles[0,:]).astype("int")

                if len(circles) == num_ball:
                    bd = bd+1
                else:
                    num_ball = len(circles)
                    bd = 0
                for (x,y,r) in circles:
                    accum_r = accum_r + r
                    r_count = r_count + 1
    avg_r = accum_r/r_count
    min_ball_size = int(avg_r) - 3
    max_ball_size = int(avg_r) + 3

    ball_data = []
    for (x,y,r) in circles:
        traj = deque()
        traj.append((x,y))
        ball_data.append(traj)
    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    max_frames = 4000

    frame_count = 0
    while(cap.isOpened() and frame_count < total_frames and frame_count < max_frames):
        #print(frame_count)
        frame_count = frame_count+1

        success, frame = cap.read()
        if success: #have a frame
            ff = np.uint8((cv2.GaussianBlur(abs(frame-background), (3,3), 2)))
            ff = cv2.cvtColor(ff, cv2.COLOR_BGR2GRAY) #HoughCircles requires greyscale image
            circles = cv2.HoughCircles(ff, cv2.HOUGH_GRADIENT, 2, 17,param1=80,param2=25, minRadius = min_ball_size, maxRadius = max_ball_size)
            circles = np.round(circles[0,:]).astype("int")
            if len(circles) != num_ball:
                continue
            for (x,y,r) in circles:
                cv2.circle(frame, (x,y),r,(0,255,0),4)
            for i in range(num_ball):
                dist = []
                traj = ball_data[i]
                prev = traj[-1]
                for (x,y,r) in circles:
                    guess = (x,y)
                    distance = ((guess[0] - prev[0])**2 + (guess[1] - prev[1])**2)**.5
                    if r == 0:
                        distance = 10000
                    dist.append(distance) 
                val, idx = min((val, idx) for (idx, val) in enumerate(dist))
                (x,y,r) = circles[idx]
                circles[idx][2] = 0
                traj.append((x,y))
        vout.write(frame)
        update_progress(frame_count/min((total_frames,max_frames)))

    while len(ball_data[0]):
        text = ""
        for traj in ball_data:
            text = text + str(traj.pop()) + ", "
        f.write(text + "\n")

    vout.release()
    vout = None
    cap.release()
    cv2.destroyAllWindows()
    f.close()
    return 

def update_progress(workdone):
    print("\rProgress: [{0:50s}] {1:.1f}%".format('#' * int(workdone * 50), workdone*100), end="", flush=True)
    



    

            
        