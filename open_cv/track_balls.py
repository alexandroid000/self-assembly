#!/usr/bin/env python

import cv2
import numpy as np
import math as m
import time

def is_in(x, y, vid_size):
    if y >= vid_size[0] or y < 0 or x < 0 or x >= vid_size[1]:
        return False
    return True


def find_background(in_vid, vid_size, write_background, save_as):
    cap = cv2.VideoCapture(in_vid)
    capSize = (vid_size[0], vid_size[1]) # this is the size of my source video
    j=0

    video_frames = np.zeros((vid_size[0], vid_size[1], 3, 100))

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
    file_name = save_as + "_background.png"
    cv2.imwrite(file_name, frame_bg)
    return frame_bg


def track(in_vid, vid_size, background, write_video, save_as):
    cap = cv2.VideoCapture(in_vid)
    fps = cap.get(5)
    capSize = (int(cap.get(3)), int(cap.get(4)))

    if write_video:
        fps = cap.get(5)
        fourcc = cv2.VideoWriter_fourcc('M','J','P','G')
        vout = cv2.VideoWriter()
        filename = save_as + '_tracking.avi'
        success = vout.open(filename,fourcc,fps,capSize,True)
    j=0
    trajectory_name = save_as + '_trajectory.txt'
    f = open(trajectory_name,'w')
    cap = cv2.VideoCapture(in_vid)

    past_positions = [[0,0],[0,0],[0,0]]
    new_positions = [[0,0],[0,0],[0,0]]

    while(cap.isOpened()):
        ret, frame = cap.read()
        if ret == True:
            ## skip first few frames due to our video having a few problem frames
            if j < 30:
                j+=1
                continue
            else:
                #initialize array determining which balls were found
                new = [False, False, False]

                #subtract background and apply blur
                frame = cv2.GaussianBlur(abs(frame-background), (3,3), 2)

                #convert data type
                frame = np.uint8(frame)

                #convert to grayscale, find circles, and round
                gray_frame = np.uint8(frame)
                gray_frame = cv2.cvtColor(gray_frame, cv2.COLOR_BGR2GRAY)
                circles = cv2.HoughCircles(gray_frame, cv2.HOUGH_GRADIENT, 2, 17,param1=80,param2=25, minRadius = 10, maxRadius = 25)
                circles = np.uint16(np.around(np.float64(circles)))
                #sort the circles by color
                not_sorted = []
                for i in circles[0,:]:
                    x = i[0]
                    y = i[1]
                    r = i[2]
                    color = np.mean(np.mean(frame[y-r/2:y+r/2, x-r/2:x+r/2,:],0),0)
                    if np.max(color) < 50.: 
                        True
                    #check for large blue component
                    elif color[0]/color[1] > 1.5 and color[0]/color[2] > 1.5:
                        color = [255,0,0]
                        new_positions[0] = np.array([x,y])
                        new[0] = True
                    #check for large green component
                    elif color[1]/color[0] > 1.5 and color[1]/color[2] > 1.5:
                        color = [0,255,0]
                        new[1] = True
                        new_positions[1] = np.array([x,y])
                    #check for large red component
                    elif color[2]/color[0] > 1.55 and color[2]/color[1] > 1.55:
                        color = [0,0,255]
                        new[2] = True
                        new_positions[2] = np.array([x,y])
                    #unsorted circles will be drawn as white
                    else:
                        color = [255,255,255]
                        not_sorted.append([x,y,r])
                    if write_video:
                        # draw the outer circle
                        cv2.circle(frame,(x,y),r,(0,255,0),2)
                        frame[y-int(r/1.414):y+int(r/1.414), x-int(r/1.414):x+int(r/1.414),:] = color

                #find which circles were not found
                not_filled = [i for i in range(3) if new[i]==False]

                #if too many unsorted for unfilled postions, combine ones that overlap
                if len(not_sorted) > len(not_filled):
                    new_not_sorted = []
                    for s in range(len(not_sorted)):
                        [x0,y0,r0] = not_sorted[s]
                        x0 = float(x0)
                        y0 = float(y0)
                        r0 = float(r0)
                        add = True
                        for t in range(len(new_not_sorted)):
                            [x1,y1,r1] = new_not_sorted[t]
                            if np.linalg.norm(np.array([x0,y0])-np.array([x1,y1])) < r0:
                                add = False
                        if add == True:
                            new_not_sorted.append([x0,y0,r0])
                    not_sorted = new_not_sorted

                #try to match remaining unsorted to unfilled positions
                if len(not_filled) > 0:
                    if len(not_sorted) == 1:
                        [x,y,r] = not_sorted[0]
                        if len(not_filled) == 1:
                            new_positions[not_filled[0]] = np.array([x,y])
                            color = [0,0,0]
                            color[not_filled[0]] = 255
                            frame[int(y-r/1.414):int(y+r/1.414), int(x-r/1.414):int(x+r/1.414),:] = color
                        else:
                            x= float(x)
                            y = float(y)
                            if is_in(x,y, vid_size):
                                d0 = np.linalg.norm(np.array([x,y]) - past_positions[not_filled[0]])
                                current_i = 0
                                for i in range(1,len(not_filled)):
                                    d1 = np.linalg.norm(np.array([x,y]) - past_positions[not_filled[i]])
                                    if d1 < d0:
                                        current_i = i
                                        d0 = d1
                                new_positions[not_filled[current_i]] = np.array([x,y])
                                color = [0,0,0]
                                color[not_filled[current_i]] = 255
                                frame[int(y-r/1.414):int(y+r/1.414), int(x-r/1.414):int(x+r/1.414),:] = color

                    elif len(not_sorted) == 2:
                        if len(not_filled) == 2:
                            [x0, y0, r0] = not_sorted[0]
                            [x1, y1, r1] = not_sorted[1]
                            past_positions = [[float(past_positions[s][t]) for t in range(2)] for s in range(3)]
                            d0 = np.linalg.norm(np.array([x0,y0]) - past_positions[not_filled[0]]) + np.linalg.norm(np.array([x1,y1]) - past_positions[not_filled[1]])
                            d1 = np.linalg.norm(np.array([x0,y0]) - past_positions[not_filled[1]]) + np.linalg.norm(np.array([x1,y1]) - past_positions[not_filled[0]])
                            if d0 < d1:
                                new_positions[not_filled[0]] = np.array([x0,y0])
                                color = [0,0,0]
                                color[not_filled[0]] = 255
                                frame[int(y0-int(r0/1.414)):int(y0+int(r0/1.414)),int(x0-int(r0/1.414)):int(x0+int(r0/1.414)),:] = color
                                new_positions[not_filled[1]] = np.array([x1,y1])
                                color = [0,0,0]
                                color[not_filled[1]] = 255
                                frame[int(y1-int(r1/1.414)):int(y1+int(r1/1.414)), int(x1-int(r1/1.414)):int(x1+int(r1/1.414)),:] = color
                            else:
                                new_positions[not_filled[1]] = np.array([x0,y0])
                                color = [0,0,0]
                                color[not_filled[1]] = 255
                                frame[int(y0-int(r0/1.414)):int(y0+int(r0/1.414)), int(x0-int(r0/1.414)):int(x0+int(r0/1.414)),:] = color
                                new_positions[not_filled[0]] = np.array([x1,y1])
                                color = [0,0,0]
                                color[not_filled[0]] = 255
                                frame[int(y1-int(r1/1.414)):int(y1+int(r1/1.414)),int(x1-int(r1/1.414)):int(x1+int(r1/1.414)),:] = color

                        else:
                            [x0, y0, r0] = not_sorted[0]
                            [x1, y1, r1] = not_sorted[1]
                            past_positions = [[float(past_positions[s][t]) for t in range(2)] for s in range(3)]
                            symmetric_group = [[0,2,1],[1,0,2],[1,2,0],[2,0,1],[2,1,0]]
                            d0 = np.linalg.norm(np.array([float(x0),y0]) - past_positions[0]) + np.linalg.norm(np.array([x1,y1]) - past_positions[1])
                            element = [0,1,2]
                            for [q,r,s] in symmetric_group:
                                d1 = np.linalg.norm(np.array([float(x0),y0]) - past_positions[q]) + np.linalg.norm(np.array([x1,y1]) - past_positions[r])
                                if d1 < d0:
                                    d0 = d1
                                    element = [q,r,s]
                            for i in range(2):
                                [x,y,r] = not_sorted[i]
                                new_positions[element[i]] = np.array([x,y])
                                color = [0,0,0]
                                color[element[i]] = 255
                                frame[int(y-int(r/1.414)):int(y+int(r/1.414)), int(x-int(r/1.414)):int(x+int(r/1.414)),:] = color  

                    elif len(not_sorted) == 3:
                        [x0, y0, r0] = not_sorted[0]
                        [x1, y1, r1] = not_sorted[1]
                        [x2, y2, r2] = not_sorted[2]
                        past_positions = [[float(past_positions[s][t]) for t in range(2)] for s in range(3)]
                        symmetric_group = [[0,2,1],[1,0,2],[1,2,0],[2,0,1],[2,1,0]]
                        d0 = np.linalg.norm(np.array([x0,y0]) - past_positions[0]) + np.linalg.norm(np.array([x1,y1]) - past_positions[1]) + np.linalg.norm(np.array([x2,y2]) - past_positions[2])
                        element = [0,1,2]
                        for [q,r,s] in symmetric_group:
                            d1 = np.linalg.norm(np.array([x0,y0]) - past_positions[q]) + np.linalg.norm(np.array([x1,y1]) - past_positions[r]) + np.linalg.norm(np.array([x2,y2]) - past_positions[s])
                            if d1 < d0:
                                d0 = d1
                                element = [q,r,s]
                        for i in range(3):
                            [x,y,r] = not_sorted[i]
                            new_positions[element[i]] = np.array([x,y])
                            color = [0,0,0]
                            color[element[i]] = 255
                            frame[int(y-int(r/1.414)):int(y+int(r/1.414)), int(x-int(r/1.414)):int(x+int(r/1.414)),:] = color

                #write output positions of balls
                #note, if a position was never filled an empty string will be written for it
                string_to_write = ""
                for pos in new_positions:
                    for coord in pos:
                        string_to_write+=str(coord)+','
                string_to_write+='\n'
                f.write(string_to_write)
                j+=1
                past_positions = new_positions
                if write_video:
                    vout.write(frame)
        else:
            break

    cap.release()
    if write_video:
        vout.release()
        vout = None
    cv2.destroyAllWindows()
    f.close()
    return 1

#interpolate so that no lines have empty positions
#(should probably produce error if trying to interpolate too far)
def interpolate(save_as):

    trajectory_x = [[],[],[]]
    trajectory_y = [[],[],[]]
    trajectory_name = save_as + '_trajectory.txt'
    f = open(trajectory_name,'r')

    for lines in f:
        l = lines.strip()
        l = l.split(',')
        for i in range(3):
            trajectory_x[i].append(float(l[2*i]))
            trajectory_y[i].append(float(l[2*i+1]))

    f.close()

    interpolate = []

    interpolated_trajectory_x = [[],[],[]]
    interpolated_trajectory_y = [[],[],[]]

    for i in range(3):
        past_x = trajectory_x[i][0]
        past_y = trajectory_y[i][0]
        interpolated_trajectory_x[i].append(past_x)
        interpolated_trajectory_y[i].append(past_y)
        for j in range(1,len(trajectory_x[0])):
            new_x = trajectory_x[i][j]
            new_y = trajectory_y[i][j]
            if new_x == past_x and new_y == past_y:
                interpolate.append([past_x, past_y])
                if j == len(trajectory_x[0]) - 1:
                    for k in range(0, len(interpolate)):
                        interpolated_trajectory_x[i].append(past_x)
                        interpolated_trajectory_y[i].append(past_y)
                    interpolate = []
            else:
                num = len(interpolate)+1
                if len(interpolate) == 0:
                    interpolated_trajectory_x[i].append(new_x)
                    interpolated_trajectory_y[i].append(new_y)
                    past_x = new_x
                    past_y = new_y
                else:
                    for k in range(1,num+1):
                        interpolated_trajectory_x[i].append(past_x+(new_x-past_x)/(1.*num)*k)
                        interpolated_trajectory_y[i].append(past_y+(new_y-past_y)/(1.*num)*k)
                    past_x = new_x
                    past_y = new_y
                    interpolate = []

    f = open(save_as + '_interpolated_trajectory.txt','w')

    for frame_num in range(len(interpolated_trajectory_x[0])):
        string_to_write = ''
        for i in range(3):
            string_to_write+= str(int(interpolated_trajectory_x[i][frame_num])) + ',' + str(int(interpolated_trajectory_y[i][frame_num])) + ','
        string_to_write += '\n'
        f.write(string_to_write)

    f.close()
