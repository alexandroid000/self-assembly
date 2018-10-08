# Group Finder
# 
# Austin Born - 10/8/18
# Main function openCVConfigFinder and the following subfunctions
# take in an array of time-series location data and output time-series
# of lexicographic labels

# Global Constants
SPACE_DIM_X
SPACE_DIM_Y
MATRIX_DIM_X
MATRIX_DIM_Y
STRAIGHT_DIST
DIST_DEVIATION


'''
openCVConfigFinder:
# Input: [t, [x0, y0], [x1, y1], [x2, y2], ..., [xn, yn]]
# OUtput Set of lexicographic labels for groups and center location of each group
'''
opencvConfigFinder(time, locationList):
    retList = []
    # Calls other subroutines
    return retList

'''
groupFinder:
# Input: [[x0, y0], [x1, y1], [x2, y2], ..., [xn, yn]]
# Output: [[x0, y0, g0], [x1, y1, g1], [x2, y2, g2], ..., [xn, yn, gn]]
'''
groupFinder(locationList):
# Build groups of bots based on parameters for straight and diagonal distance
    retList = []
    return retList
# Create new list for each bot and set "groups" to None
# For each bot, run through found bot group list, and if bot is
# within distance of any of the existing groups, add it to that group.
# Otherwise, create new group for it.


'''
centerpointFinder:
# Input: [[x0, y0], [x1, y1], [x2, y2], ..., [xn, yn]]
# Output: [x, y]
'''
centerpointFinder(locationList):
# Find center location of a group
    center = (0, 0)
    return center


'''
cartTransform:
# Input: [[x0, y0], [x1, y1], [x2, y2], ..., [xn, yn]]
# Output: [Matrix of values based on lexLabel input]
'''
cartTransform(locationList):
# Find a straight distance between bots
# Use matrix transformation to align with cartesian plane
# Re-build plane as input to polyomino generation lexLabelOrdering



'''
lexLabelOrdering:
# Input: [Matrix of values based on lexlabel input]
# Output: [List of lex labels]
'''
# Already given in PolyominoGenerator.py


'''
groupFixer:
# Input [t, List of lex labels]
# Output [t, List of updated lex labels]
'''
groupFixer():
# Take output and use time-reversed analysis to confirm time of actual linking
