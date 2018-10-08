#!/usr/bin/env python

import sys
import tracking as tb
#import winding_numbers as wn
import os
import argparse
import threading


def main():
    #handling arguements
    parser = argparse.ArgumentParser(prog='Ball trajectory detecter', description='Process some weaselball video data')
    parser.add_argument('v', metavar='video', type=str, nargs='+')
    args = parser.parse_args()

    tb.track(args.v[0])
    print("Done!")


main()
