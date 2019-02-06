# Group Finder
# 
# Austin Born - 10/12/18
# Main function openCVConfigFinder and the following subfunctions
# take in an array of time-series location data and output time-series
# of lexicographic labels

# Graph-based approach investigation? 
# Image matching approach (registration problem)

# TODO
# Time-series filtering for groups vs. single units
# Investigate python libraries for matrix transformations

# Import modules
import sys
import numpy as np
import math
from os.path import join
import re
import matplotlib.pyplot as plt


# Global Constants
UNIT_CONNECT_DIST  = 15  # Dimension of unit hub in pixels
SPACE_DIM_X        = 600 # Dimension of space width in pixels
SPACE_DIM_Y        = 470 # Dimension of space height in pixels


'''
groupFinder:
# Input: [[x0, y0], [x1, y1], [x2, y2], ..., [xn, yn]]
# Output: [[[x0, y0], [x1, y1], ..., [xn, yn]], [[xn+1, yn+1],...,[xm, ym]], [...]]
'''
def groupFinder(locationList):
    # Initializations
    groupList = [[locationList[0]]]
    unit_added = False

    # For each unit in location list
    for unit in locationList:

        unit_added = False

        # For each group
        for group in groupList:

            # For each unit in group
            for unit_other in group:

                # If same unit, continue to next group
                if unit is unit_other:
                    unit_added = True
                    break

                # If unit is within distance of group member, append
                dist = distance(unit, unit_other)
                if (dist < UNIT_CONNECT_DIST):
                    group.append(unit)
                    unit_added = True
                    break

            if unit_added:
                break

        if not unit_added:
            groupList.append([unit])

    return groupList


'''
distance:
# Input: (point1, point2)
# Output: distance as nearest int
'''
def distance(point1, point2):
    return math.sqrt((point1[0]-point2[0])**2 + (point1[1]-point2[1])**2)


'''
centerpointFinder:
# Input: [[x0, y0], [x1, y1], [x2, y2], ..., [xn, yn]]
# Output: (x, y)
'''
def centerpointFinder(locationList):
    # Find center location of a group
    centerX, centerY = (0.0, 0.0)
    unitNum = len(locationList)
    for unit in locationList:
        centerX += unit[0]
        centerY += unit[1]
    centerX /= unitNum
    centerY /= unitNum
    return (centerX, centerY)


'''
cartTransform:
# Input: [[x0, y0], [x1, y1], [x2, y2], ..., [xn, yn]]
# Output: [Matrix of values based on lexLabel input]
'''
def cartTransform(locationList):
# Find a straight distance between bots
# Use matrix transformation to align with cartesian plane
# Re-build plane as input to polyomino generation lexLabelOrdering
    locationList = np.array(locationList)
    unit_one = locationList[0]
    unit_two = None
    for unit in locationList:
        dist = distance(unit, unit_one)
        if (dist < UNIT_DIM + DIST_DEVIATION) and (dist > UNIT_DIM - DIST_DEVIATION):
            unit_two = unit
            break
    if unit_two is not None:
        # Compute angle in radians
        angle = -math.acos(float(unit_two[0] - unit_one[0])/distance(unit_one, unit_two))
        rotMatrix = [[math.cos(angle), -math.sin(angle)], [math.sin(angle), math.cos(angle)]]
        locationList = np.matmul(rotMatrix, np.transpose(locationList))
        locationList = np.transpose(locationList)
    (xd, yd) = (0, 0)
    for unit in locationList:
        if unit[0] < xd:
            xd = unit[0]
        if unit[1] < yd:
            yd = unit[1]
    transMatrix = [[1-xd, 1-yd] for i in range(len(locationList))]
    locationList += transMatrix

    locationList = np.divide(locationList, UNIT_DIM)
    locationList = np.floor(locationList)

    return locationList.astype(int)


'''
lexLabelOrdering:
# Input: [[[x0, y0], [x1, y1], ..., [xn, yn]], [[xn+1, yn+1],...,[xm, ym]], [...]]
# Output: [List of lex labels]
'''
# lexLabelOrdering() takes the matrix and returns the proper trinary label
    # for a given configuration with the highest lexicographic ordering. Starting from
    # the upperleft corner point of the top-left block, starting by facing the right direction, 
    # the label is as follows:
    # 0 - straight
    # 1 - right
    # 2 - left   
    # The lexicographic label is found after the clockwise path is "traversed".
def lexLabelOrdering(groupList):
    labelsList = []
    matrix = [[0 for i in range (SPACE_DIM_X)] for j in range (SPACE_DIM_Y)]
    for group in groupList:
        for unit in group:
            matrix[unit[0]][unit[1]] = 'X'


    for group in groupList:
        label = '1'
        direction = 'E'
        (startx, starty) = (group[0][0], group[0][1])
        
        # Set up start location
        for unit in group:
            if unit[0] < startx:
                startx = unit[0]
                starty = unit[1]
            elif unit[0] == starty:
                if unit[1] < startx:
                    starty = unit[1]
            
        # Start path after start value
        (pathx, pathy) = (startx+1, starty)

        # While loop to create label
        while not (pathx == startx and pathy == starty):
            if direction == 'E':
                if matrix[pathy-1][pathx] == 'X':
                    # Go left
                    label += '2'
                    direction = 'N'
                    pathy -= 1
                    continue
                if matrix[pathy][pathx] == 0:
                    # Go straight
                    label += '0'
                    pathx += 1
                    continue
                # Else, go right
                label += '1'
                direction = 'S'
                pathy += 1

            elif direction == 'S':
                if matrix[pathy][pathx] == 'X':
                    # Go left
                    label += '2'
                    direction = 'E'
                    pathx += 1
                    continue
                if matrix[pathy][pathx-1] == 'X':
                    # Go straight
                    label += '0'
                    pathy += 1
                    continue
                # Else, go right
                label += '1'
                direction = 'W'
                pathx -= 1

            elif direction == 'W':
                if matrix[pathy][pathx-1] == 'X':
                    # Go left
                    label += '2'
                    direction = 'S'
                    pathy += 1
                    continue
                if matrix[pathy-1][pathx-1] == 'X':
                    # Go straight
                    label += '0'
                    pathx -= 1
                    continue
                # Else, go right
                label += '1'
                direction = 'N'
                pathy -= 1

            else: #direction == 'N'
                if matrix[pathy-1][pathx-1] == 'X':
                    # Go left
                    label += '2'
                    direction = 'W'
                    pathx -= 1
                    continue
                if matrix[pathy-1][pathx] == 'X':
                    # Go straight
                    label += '0'
                    pathy -= 1
                    continue
                # Else, go right
                label += '1'
                direction = 'E'
                pathx += 1

        # Iteratively shift the label values to 
        # find the true lexicographic label
        labelVal = int(label)
        labelChars = list(label)
        for i in range(len(labelChars)):
            labelChars.insert(0, labelChars.pop())
            if int(''.join(labelChars)) > labelVal:
                labelVal = int(''.join(labelChars))

        labelsList.append(labelVal)
                
    return labelsList


'''
groupFixer:
# Input [{t0, List of lex labels}, {t1, List of lex labels}, ..., {tn, List of lex labels}]
# Output [{t0, List of lex labels}, {t1, List of lex labels}, ..., {tn, List of lex labels}]
'''
def groupFixer():
# External function to take output and use time-reversed analysis to confirm time of actual linking
    return 0


# Main functions testing
if __name__ == '__main__':

    print("Testing trajectory data")

    # Parse position data
    f = open(sys.argv[1])
    timer = -1
    for line in f:
        timer += 1
        if timer == 140:
                break
        numbers = re.findall('\d+', line)
        positions = []
        for xandy in range(math.floor(len(numbers)/2)):
            positions.append([int(numbers[2*xandy]), int(numbers[2*xandy+1])])
        print("Positions: ", positions)

        # set up the figure for all
        if (timer%2 == 0) and (timer < 80): #29 fps
            fig = plt.figure()
            ax = fig.add_subplot(111)
            ax.set_xlim(0,640)
            ax.set_ylim(0,480)

            # draw lines
            xmin = 0
            xmax = 640
            ymin = 0
            ymax = 480
            plt.hlines((ymin,ymax), xmin, xmax)
            plt.vlines((xmin,xmax), ymin, ymax)

            # draw units
            for unit in positions:
                plt.plot(unit[0], ymax-unit[1], 'ko', ms = 10, mfc = 'k')


            plt.axis('off')
            figname = 'group'+str(timer)+'.png'
            plt.savefig(figname)

        # Calculate distances
        for i in range(len(positions)):
            for j in range(i+1, len(positions)):
                print(distance(positions[i], positions[j]))

        # Find groups
        groupLists = groupFinder(positions)
        # print("Groups: ", groupLists)
        
        # Place all on different pics
        for idx in range(len(groupLists)):

            if timer == 141:

                # set up the figure
                fig = plt.figure()
                ax = fig.add_subplot(111)
                ax.set_xlim(0,640)
                ax.set_ylim(0,480)

                # draw lines
                xmin = 0
                xmax = 640
                ymin = 0
                ymax = 480
                plt.hlines((ymin,ymax), xmin, xmax)
                plt.vlines((xmin,xmax), ymin, ymax)

                # draw units
                for unit in groupLists[idx]:
                    plt.plot(unit[0], unit[1], 'ko', ms = 10, mfc = 'k')


                plt.axis('off')
                figname = 'group'+str(idx+1)+".png"
                plt.savefig(figname)







            # center = centerpointFinder(groupLists[idx])
            # # print("Center: ", center)
            # if(len(groupLists[idx]) > 1):
            #     # print("Cart transform on: ", groupLists[idx])
            #     groupLists[idx] = cartTransform(groupLists[idx])
            #     # print("Post-transform: ", groupLists[idx])

            
    # # Find labels
    # print(groupLists)
    # retLabels = lexLabelOrdering(groupLists)
        
    # print(retLabels)
