#!/usr/bin/env python

import sys
import tracking as tb
import queue
#import winding_numbers as wn
import os
import argparse
import threading
import magic
import multiprocessing as mp
import tqdm

def main():
    #handling arguements
    parser = argparse.ArgumentParser(prog='Ball trajectory detecter', description='Process some weaselball video data')
    
    parser.add_argument('locs', 
    help='Location of target file(s) -- file(s) for regular mode, directory(s) for batch mode',
    metavar='locs', 
    type=str, 
    nargs='+')

    parser.add_argument('-b', '--batch', 
    help='Enable directory input (default: off)',
    action='store_true', 
    default='False',
    dest='batch_mode')


    parser.add_argument('--cores', 
    help='Number of cores (only applicable when more than one video files are provided) (default: CPU cores - 1)',
    action='store', 
    default= mp.cpu_count()-1,
    dest='cores')

    parser.add_argument('-o', '--output_location',
    help='Output directory (default: ./)',
    action='store', 
    default= "",
    dest='destination')

    parser.add_argument('-O', '--output_name',
    help='Output data file name (default: $(INPUT_FILE_NAME)_trajectory.txt)',
    action='store', 
    default= "",
    dest='save_name')

    args = parser.parse_args()


    #generate work list
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


    # CURRENTLY UNDER DEVELOPMENT
    # #generate total number of frames (for progress bar)
    # total_frames = tb.totalFrames(video_queue)

    #generate and wrap parameters for worker pool
    parameters = []
    while(len(video_queue) is not 0):
        vid = video_queue.pop()
        save_destination = os.getcwd() + '/' + args.destination
        if args.save_name == "":
            (directory, name) = os.path.split(vid)
            name = os.path.splitext(name)[0] + "_trajectory.txt"
        else:
            name = args.save_name
        parameters.append((vid, save_destination  + name, args.batch_mode))
    
    #create worker pool and distribute jobs
    pool = mp.Pool(processes = args.cores)

    #execute with overall progress bar
    if(len(parameters)>1):
        for _ in tqdm.tqdm(pool.imap_unordered(tb.track, parameters), total=len(parameters)):
            pass
    #execute with file progress bar
    else:
        pool.imap_unordered(tb.track, parameters)
    
    pool.close()
    pool.join()

if __name__ == "__main__":
    main()
