import sys
import track_balls as tb
import winding_numbers as wn
import cv2
import os
def main(in_vid):
    cap = cv2.VideoCapture(in_vid)
    vid_size = (int(cap.get(4)), int(cap.get(3)))
    save_name = os.path.splitext(in_vid)[0]
    print "finding background..."
    back = tb.find_background(in_vid, vid_size, False, save_name)
    print "tracking balls..."
    tb.track(in_vid, vid_size, back, True,save_name)
    print "interpolating..."
    tb.interpolate(save_name)
    print "computing first order winding numbers..."
    wn.first_order(True, save_name)
    print "computing second order winding numbers..."
    wn.second_order(True,save_name)
    return 1
    

in_vid = sys.argv[1]    
main(in_vid)
