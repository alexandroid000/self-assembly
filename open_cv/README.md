The purpose of the code in this directory is to extract trajectory data from weasel ball videos.

Trajectory data is output in the form

```
Ball1X Ball1Y Ball2X Ball2Y Ball3X Ball3Y
```

#### Dependencies

- Python3 
- openCV-python 

### Getting Started

To extract data from `inputVideo.mp4`, use `weasel_balls.py`:

```
python weasel_balls.py inputVideo.mp4
```

You can also receiving more information by using

```
python weasel_balls.py -h
```

An example video file, `4Ball-TRIM.mp4`, is included in the Videos file. 
 
### Contributors

Code originally obtained from [Yuliy Baryshnikov](https://publish.illinois.edu/ymb/).

Other contributors: Chris Horn, Austin Born, John Born
