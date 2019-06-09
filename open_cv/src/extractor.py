import numpy
import cv2
from scipy.spatial.distance import cdist
from copy import deepcopy
import multiprocessing as mp


import logging

STARTING_MIN_RAD = 10
STARTING_MAX_RAD = 20
RAD_RANGE = 3

logger = logging.getLogger(__name__)
logging.basicConfig(level = logging.WARNING)


class Extractor(object):

    # input_clip: file location of video
    # auto_gen_background: flag for generating background on construction
    # write: file location for trajectory data (none for no save)
    def __init__(self, input_clip, start_frame=0, end_frame=0,auto_gen_background=False, write=None, pipe=None):
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
            self.end_frame = self.total_frames

        if self.start_frame > self.end_frame:
            logger.warning("End frame before start frame, end defaulting to last")
            self.end_frame = self.total_frames

        cap.release()

        self.background = None
        self.min_rad = int(STARTING_MIN_RAD)
        self.max_rad = int(STARTING_MAX_RAD)
        self.ball_count = None

        self.frames = []

        if auto_gen_background:
            self.gen_background()

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

    def send_frames_pipe(self, pipe):
        pipe.send(self.frames)

    def recv_frames_pipe(self, pipe):
        self.frames = pipe.recv()

    def find_ball_count(self):
        r_accum = 0.0
        r_count = 0
        frames_scene = 0

        consensus_frames = 0
        ball_count = 0

        cap = self._get_cap()
        while consensus_frames < 15:
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
        succuess, frame = cap.read()
        if not succuess:
            raise ValueError("Failed to get frame while getting circles")
            return None
        ff = numpy.uint8((cv2.GaussianBlur(abs(frame-self.background), (3,3), 2)))
        ff = cv2.cvtColor(ff, cv2.COLOR_BGR2GRAY)
        circles = cv2.HoughCircles(ff, cv2.HOUGH_GRADIENT, 2, 17, 
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
            
    # get list of cirlces
    # match with previous, then 
    # write to dictionary and push to frames
    def write_frame(self, frame):
        self.frames.append(frame)

    def _set_first_frame(self, circles):
        first_dict = {}
        base_name = "Ball "
        for idx in range(len(circles)):
            name = base_name + str(idx + 1)
            first_dict[name] = [circles[idx][0], circles[idx][1]]
        self.write_frame(first_dict)

    def strip_radius(self, circles):
        points = []
        
        for (x,y,r) in circles:
            points.append([x,y])
        return points

    def to_list(self):
        frame_list = []

        try:
            ball_keys = self.frames[0].keys()
        except ValueError:
            logger.warning("Frames empty when converted to list")
            return frame_list


        for frame in self.frames:
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
        logger.debug("Generated ball count")
        # create 
        
        cap = self._get_cap()
        circles = self.get_circles(cap)   

        self.frames = []

        if len(self.frames) is 0:
            self._set_first_frame(circles)

        try:
            while cap.isOpened() and cap.get(1) < self.end_frame:
                circles = self.get_circles(cap)
                if circles is None:
                    break
                points = self.strip_radius(circles)
                frame = self.match_labels(self.frames[-1], points)
                self.write_frame(frame)
        except ValueError as error:
            #end of file?
            pass

        cap.release()


class Trajectory(object):

    def __init__(self, file, threads=0):
        self.input_file = file
        self.extractors = []
        self.threads = 0.0
        self.frames = []

        if threads == 0:
            self.threads = mp.cpu_count() - 1
        else:
            self.threads = threads
        
        

        
    def _run_extractor(self, file, start, end, pipe):
        e = Extractor(file, start_frame=start, end_frame=end)
        e.extract()
        pipe.send((start, e.get_frames()))        

    def extract_trajectory(self):
        jobs = []
        frame_blocks = []
        for th in range(self.threads):
            start = th / self.threads
            end = (th + 1) / self.threads
            send, recv = mp.Pipe()

            p = mp.Process(target=self._run_extractor, 
                            args = (self.input_file, start, end, send))
            jobs.append((p, recv))
            p.start()
        
        for (proc, pipe) in jobs:
            frame_blocks.append(pipe.recv())
            proc.join()

        frame_blocks.sort(key=lambda blk: blk[0])

        for blk in frame_blocks:
            self.frames += blk[1]


        

    def get_frames(self):
        return self.frames
        


if __name__ == "__main__":
    t = Trajectory("../SampleVideos/4B-ML-1.mp4", threads=4)
    t.extract_trajectory()
    print(len(t.get_frames()))