The purpose of the code in this directory is to extract trajectory data from weasel ball videos.

Trajectory data is output in the form

```
Ball1X Ball1Y Ball2X Ball2Y ... BallnX BallnY
```

#### Dependencies

- Python3 

Python libraries:

- opencv, version 3 or greater
- python3-magic

### Getting Started

The folder SampleVideos contains a collection of short practice videos. wba.py, which stands for Weasel Ball Analysis, is used to extract the trajectories from the input video files.

To extract data from `inputVideo.mp4`, use `wba.py`:

```
python wba.py inputVideo.mp4
```

To extract data from all the video files in a folder, use the batch -b flag:

```
python wba.py -b /path/to/folder
```

By default, multithreading is enabled. However, it is only used for jobs with more than one input file. It can be disabled with the flag ```--cores 0```

Additional flags can be used for settings file suffixes for batch naming, as well as output folders and core count selection. You can also receiving more information by using

```
python wba.py -h
```
 
### Contributors

Code originally obtained from [Yuliy Baryshnikov](https://publish.illinois.edu/ymb/).

Other contributors: Chris Horn, Austin Born, John Born
