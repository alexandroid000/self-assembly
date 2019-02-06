#!/usr/bin/env python

import numpy as np
import cv2
import argparse
from collections import deque
import sys

min_ball_size = 10
max_ball_size = 20



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


def track(parameters):

    in_vid = parameters[0]
    save_location = parameters[1]
    status_bar = parameters[2]
    verbose = parameters[3]

    #determine size and frame rate of input video
    cap = cv2.VideoCapture(in_vid)
    fps = cap.get(cv2.CAP_PROP_FPS)
    capSize = (int(cap.get(3)), int(cap.get(4)))
    #total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    total_frames = 100
    vout = cv2.VideoWriter()

    #current forced off due to codec limitations
    #may be optional at the later date, or can be used if opencv 
    #is compiled from source, and all necessary codecs are installed

    #save the output video
    write = False
    if write:
        fourcc = cv2.VideoWriter_fourcc(*'avc1')
        filename = 'tracking.mp4'
        success = vout.open(filename,fourcc,fps,capSize,True)

    #open output file
    f = open(save_location, 'w+')


    #find background image
    background = find_background(in_vid)

    #initial ball sizes
    min_ball_size = 10
    max_ball_size = 20


    num_ball, avg_r = find_ball_count(cap, background)

    min_ball_size = int(avg_r) - 3
    max_ball_size = int(avg_r) + 3

    #releasing and reopening to get the first frame
    cap.release()
    cap = cv2.VideoCapture(in_vid)

    #trajectory path
    ball_data = []

    #pull single frame and get initial locations for the balls
    success, frame = cap.read()
    circles = get_circles(frame, background)
    for (x,y,r) in circles:
        traj = deque()
        traj.append((x,y))
        ball_data.append(traj)
    
    #loop through video
    frame_count = 0
    while(cap.isOpened() and frame_count < total_frames):
        frame_count = frame_count+1
        success, frame = cap.read()

        if success: #have a frame
            circles = get_circles(frame, background)

            if len(circles) != num_ball:
                continue
            if write:
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
        if write:
            vout.write(frame)
        if(parameters[2] == False):
            update_progress(frame_count/total_frames)


    #write data to text file
    while len(ball_data[0]):
        text = ""
        for traj in ball_data:
            text = text + str(traj.pop()) + ", "
        f.write(text + "\n")

    #clean up
    vout.release()
    vout = None
    cap.release()
    cv2.destroyAllWindows()
    f.close()
    return 

def update_progress(workdone):
    print("\rProgress: [{0:50s}] {1:.1f}%".format('#' * int(workdone * 50), workdone*100), end="", flush=True)

def find_ball_count(cap, background):
    #finding average ball radius to improve detection
    accum_r = 0.0
    r_count = 0.0 

    bd = 0 #balls detected in given frame (used to find number of balls)
    num_ball = 0


    while bd < 15:
        success, frame = cap.read()

        if success: #have a frame

            #Frame preprocessing
            ff = np.uint8((cv2.GaussianBlur(abs(frame-background), (3,3), 2))) #remove background and remove high frequency noise
            ff = cv2.cvtColor(ff, cv2.COLOR_BGR2GRAY) #HoughCircles requires greyscale image

            #detect all circles in frame
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

    return num_ball, avg_r



def get_circles(frame, background):
    ff = np.uint8((cv2.GaussianBlur(abs(frame-background), (3,3), 2)))
    ff = cv2.cvtColor(ff, cv2.COLOR_BGR2GRAY) #HoughCircles requires greyscale image
    circles = cv2.HoughCircles(ff, cv2.HOUGH_GRADIENT, 2, 17,param1=80,param2=25, minRadius = min_ball_size, maxRadius = max_ball_size)
    circles = np.round(circles[0,:]).astype("int")
    return circles

def countFrames(vq):
    frame_count = 0
    for vid in tqdm.tqdm(vq):
        cap = cv2.VideoCapture(vid)
        frame_count += int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    return frame_count


    



    

            
        
