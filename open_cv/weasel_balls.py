#!/usr/bin/env python

import sys
import tracking as tb
import queue
#import winding_numbers as wn
import os
import argparse
import threading
import magic
from multiprocessing import pool
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
    dest='cores')

    args = parser.parse_args()

    video_queue = []

    if(args.batch_mode == True):
        for folder in args.locs:
            for file in os.listdir(folder):
                with magic.Magic(flags=magic.MAGIC_MIME_TYPE) as m:
                    name = str('./'+ folder + '/' + file)
                    file_type = (m.id_filename(name))
                    if(file_type[0:5] == "video"):
                        video_queue.append(name)

    else:
        for file in args.locs:
            video_queue.append(file)

    parameters = []
    while(len(video_queue is not 0)):
        vid = video_queue.pop()
        folder = os.path.dirname(vid)
        parameters.append(vid, vid + "_trajectory.txt")
    
    pool = mp.Pool(processes = args.cores)
    pool.map(tb.tracking, parameters)

    

            



if __name__ == "__main__":
    main()
