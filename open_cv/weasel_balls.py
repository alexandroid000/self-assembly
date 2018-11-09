#!/usr/bin/env python

import sys
import tracking as tb
import queue
#import winding_numbers as wn
import os
import argparse
import threading
import multiprocessing as mp
import magic


def main():
    #handling arguements
    parser = argparse.ArgumentParser(prog='Ball trajectory detecter', description='Process some weaselball video data')
    
    parser.add_argument('locs', 
    metavar='locs', 
    type=str, 
    nargs='+')

    parser.add_argument('-b', '--batch', 
    action='store_true', 
    default='False',
    dest='batch_mode')


    parser.add_argument('--cores', 
    action='store', 
    default= mp.cpu_count()-1,
    dest='core_count')

    args = parser.parse_args()

    video_queue = queue.Queue()

    if(args.batch_mode == True):
        for folder in args.locs:
            for file in os.listdir(folder):
                with magic.Magic(flags=magic.MAGIC_MIME_TYPE) as m:
                    name = str('./'+ folder + '/' + file)
                    file_type = (m.id_filename(name))
                    if(file_type[0:5] == "video"):
                        video_queue.put(name)

    else:
        for file in args.locs:
            video_queue.put(file)
    print(video_queue.qsize())
    while(not video_queue.empty()):
        vid = video_queue.get()
        folder = os.path.dirname(vid)
        print("Processing: " + vid)
        tb.track(vid, vid + "_trajectory.txt")
            



if __name__ == "__main__":
    main()
