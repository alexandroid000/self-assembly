import numpy
import cv2
from scipy.spatial.distance import cdist
from copy import deepcopy
import multiprocessing as mp
from tqdm import tqdm
from itertools import repeat



import logging

STARTING_MIN_RAD = 10
STARTING_MAX_RAD = 20
RAD_RANGE = 3

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)

ch = logging.StreamHandler()
ch.setLevel(logging.WARNING)

fh = logging.FileHandler("debug.log")
fh.setLevel(logging.DEBUG)

formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
fh.setFormatter(formatter)
ch.setFormatter(formatter)

logger.addHandler(fh)
logger.addHandler(ch)




class Extractor(object):

    # input_clip: file location of video
    # auto_gen_background: flag for generating background on construction
    # write: file location for trajectory data (none for no save)
    def __init__(self, input_clip, start_frame=0, end_frame=0, background=False, write=None, ball_count=None):
        logger.info("Creating extractor with input: {}; start_frame: {}, end_frame: {}".format(input_clip, start_frame, end_frame))
        self.input_file = input_clip
        self.write = write

        # cap should be captured and released for each use
        # otherwise, may not be at beginning of capture, 
        # properly initialized, etc
        cap = cv2.VideoCapture(self.input_file)

        self.size = (int(cap.get(3)), int(cap.get(4)))
        self.fps = cap.get(cv2.CAP_PROP_FPS)
        self.total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))

        if start_frame >= self.total_frames:
            logger.warning("Illegal start frame, defaulting to 0")
            self.start_frame = 0
        # support 0-1 range converted to frame_num
        if start_frame > 0 and start_frame < 1:
            self.start_frame = int(self.total_frames * start_frame)
        else:
            self.start_frame = start_frame


        if end_frame >= self.total_frames:
            logger.warning("Illegal end frame, defaulting to last frame")
            self.start_frame = 0
        # support 0-1 range converted to frame_num
        if end_frame > 0 and end_frame < 1:
            self.end_frame = int(self.total_frames * end_frame)
        else:
            self.end_frame = self.total_frames-1

        if self.start_frame > self.end_frame:
            logger.warning("End frame before start frame, end defaulting to last")
            self.end_frame = self.total_frames-1

        cap.release()

        self.background = background
        self.min_rad = int(STARTING_MIN_RAD)
        self.max_rad = int(STARTING_MAX_RAD)
        self.ball_count = ball_count 

        self.frames = {}


    # TODO: Improve accuracy and quality of background generation
    def gen_background(self):
        cap = self._get_cap()
        video_frames = numpy.zeros((self.size[1], self.size[0], 3, 100))

        j = 0
        while cap.isOpened():
            ret, frame = cap.read()
            if ret:
                if j < 150:
                    j += 1
                    continue
                elif j < 250:
                    video_frames[:,:,:,j-150] = frame
                    j += 1
                else:
                    break
            else:
                break
        cap.release()

        self.background = numpy.median(video_frames,3)
        return self.background

    # returns Videocapture object, pointing at 
    # start_frame
    def _get_cap(self):
        cap = cv2.VideoCapture(self.input_file)
        cap.set(1, self.start_frame)
        return cap


    def find_ball_count(self):
        r_accum = 0.0
        r_count = 0
        frames_scene = 0

        consensus_frames = 0
        ball_count = 0

        cap = self._get_cap()
        while consensus_frames < 15 and cap.get(1) < self.end_frame:
            frames_scene += 1
            try:
                circles = self.get_circles(cap)
            except ValueError as err:
                raise
            if circles is not None:
                logger.debug("Circles found during ball count calculation on frame {}: {}"
                                .format(frames_scene, circles))
                logger.debug("Len of circles: {}".format(len(circles)))
                logger.debug("Previous ball count: {}".format(ball_count))
                circles = numpy.round(circles).astype("int")
                if len(circles) == ball_count:
                    consensus_frames += 1
                else:
                    ball_count = len(circles)
                    consensus_frames = 0
                for (x,y,r) in circles:
                    r_accum += r
                    r_count += 1
                logger.debug("Number of frames scene: {}".format(frames_scene))
                
        avg_r = r_accum/r_count

        self.radius = avg_r
        self.ball_count = ball_count
        self.min_rad = int(numpy.around(avg_r) - RAD_RANGE)
        self.max_rad = int(numpy.around(avg_r) + RAD_RANGE)

        return ball_count, avg_r


    # Returns frame, and iterates to next
    # NOTE: modifies cap by advancing a frame
    def get_circles(self, cap):
        logger.debug("Found circles on frame: {}", cap.get(1))
        succuess, frame = cap.read()
        if not succuess:
            return None
        ff = numpy.uint8((cv2.GaussianBlur(abs(frame-self.background), (3,3), 2)))
        ff = cv2.cvtColor(ff, cv2.COLOR_BGR2GRAY)
        circles = cv2.HoughCircles(ff, cv2.HOUGH_GRADIENT, 2, 15, 
                                    param1=80, param2=25, 
                                    minRadius = STARTING_MIN_RAD,
                                    maxRadius = STARTING_MAX_RAD)

        return circles.tolist()[0]

    def match_labels(self, prev_dict, next_list):
        next_dict = {}
        for ball, point in prev_dict.items():
            distance = cdist([point], next_list)
            idx = distance.argmin()
            next_dict[ball] = next_list[idx]

        return next_dict
        

    def _set_first_frame(self, circles):
        first_dict = {}
        base_name = "Ball "
        for idx in range(len(circles)):
            name = base_name + str(idx + 1)
            first_dict[name] = [circles[idx][0], circles[idx][1]]
        self.frames[self.start_frame] = first_dict

    def strip_radius(self, circles):
        points = []
        
        for (x,y,r) in circles:
            points.append([x,y])
        return points

    def to_list(self):
        frame_list = []

        try:
            ball_keys = self.frames[self.start_frame].keys()
        except ValueError:
            logger.warning("Frames empty when converted to list")
            return frame_list


        for frame_no, frame in sorted(self.frames.items(), key=lambda item: item[0]):
            data_list = []
            for k in ball_keys:
                data_list.append(frame[k])
            frame_list.append(data_list)
        return frame_list

    def get_frames(self):
        return self.frames

    def extract(self):
        if self.background is None:
            self.gen_background()

        logger.debug("Generated background")

        # generates important information about the video
        if self.ball_count is None:
            self.find_ball_count()
        logger.info("Generated ball count")
        # create 
        
        cap = self._get_cap()
        circles = self.get_circles(cap)   

        self.frames = {}

        if len(self.frames.keys()) is 0:
            self._set_first_frame(circles)

        prev_frame = self.frames[self.start_frame]
        try:
            while cap.isOpened() and (cap.get(1) < self.end_frame):
                frame_count = cap.get(1)
                circles = self.get_circles(cap)
                if circles is None:
                    continue
                points = self.strip_radius(circles)
                frame = self.match_labels(prev_frame, points)
                self.frames[frame_count] = frame
                prev_frame = frame
        except ValueError as error:
            print("Extract Error!")
            pass

        cap.release()


class Trajectory(object):

    def __init__(self, file, threads=0):
        self.input_file = file
        self.extractors = []
        self.threads = 0.0
        self.frames = {}

        self.chunks = 100

        if threads == 0:
            self.threads = mp.cpu_count() - 1
        else:
            self.threads = threads
        
        
    def _run_extractor(self, args):
        f, start, end, balls, in_background = args
        e = Extractor(f, start_frame=start, end_frame=end, ball_count=balls, background=in_background)
        try:
            e.extract()
        except:
            print("failed!")
        return e.get_frames()

    def extract_trajectory(self):
        # jobs = []
        # frame_blocks = []
        # for th in range(self.threads):
        #     start = th / self.threads
        #     end = (th + 1) / self.threads

        #     p = mp.Process(target=self._run_extractor, 
        #                     args = (self.input_file, start, end, send))
        #     jobs.append((p, recv))
        #     p.start()
        
        # for (proc, pipe) in jobs:
        #     frame_blocks.append(pipe.recv())
        #     proc.join()

        # frame_blocks.sort(key=lambda blk: blk[0])

        # for blk in frame_blocks:
        #     self.frames += blk[1]
        
        e = Extractor(self.input_file)
        logger.info("Now extracting background information")
        e.gen_background()
        logger.info("Now extracting ball count information")
        ball_count, _ = e.find_ball_count()
        background = e.background

        parameters = []
        for th in range(self.chunks):
            start = th / self.chunks
            end = (th + 1) / self.chunks
            parameters.append((self.input_file, start, end, ball_count, background))
        logger.info("Done, now starting chunk workers")
        with mp.Pool(processes=self.threads) as pool:
            with tqdm(total=self.chunks) as pbar:
                for data in pool.imap_unordered(self._run_extractor, parameters):
                    self.frames.update(data)
                    pbar.update()

    def get_frames(self):
        return self.frames
        


if __name__ == "__main__":
    t = Trajectory("../SampleVideos/4Ball-short1.mp4", )
    t.extract_trajectory()
    print(len(t.get_frames()))
