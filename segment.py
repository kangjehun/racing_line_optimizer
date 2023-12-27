import sys
import numpy as np


class Segment:
    """
    Stores the segment (straight & corner) info
    """

    def __init__(self, corners_2d, straight_2d, is_closed):
        self.is_closed = is_closed
        self.corners_2d = corners_2d
        self.straight_2d = straight_2d
        self.corner_size = 0 if corners_2d[0].size == 0 else corners_2d.shape[0]
        self.straight_size = 0 if straight_2d[0].size == 0 else straight_2d.shape[0]
        self.start = []
        self.end = []
        self.segment_type = ""
        self.analyze_segment()

    def analyze_segment(self):
        c_arr = self.corners_2d
        s_arr = self.straight_2d
        c_size = self.corner_size
        s_size = self.straight_size
        start = np.array([], dtype=int)  # set empty np array
        end = np.array([], dtype=int)  # set empty np array

        if c_size == 0:
            if s_size != 1:
                print("number of straight size : ", s_size)  # [DEBUG]
                sys.exit("there is no corner but the number of straight size is not 1")
            else:
                if(self.is_closed == True):
                    sys.exit("Track with one straight segment cannot be a closed track")
                start = np.append(start, int(s_arr[0][0]))  # starting point of straight segment
                end = np.append(end, int(s_arr[0][1]))  # ending point of straight segment
                self.segment_type = "s"  # store the segment type
        elif s_size == 0:
            if c_size != 1:
                print("number of corner size : ", c_size)  # [DEBUG]
                sys.exit("there is no straight but the number of corner size is not 1")
            else:
                start = np.append(start, int(c_arr[0][0]))  # starting point of corner segment
                end = np.append(end, int(c_arr[0][1]))  # ending point of corner segment
                self.segment_type = "c"  # store the segment type
        elif c_size == s_size:
            start_is_straight = self.check_start_is_straight()
            if start_is_straight:
                for i in range(s_size):
                    start = np.append(start, [int(s_arr[i][0]), int(c_arr[i][0])])
                    end = np.append(end, [int(s_arr[i][1]), int(c_arr[i][1])])
                self.segment_type = "sc"
            else:
                for i in range(c_size):
                    start = np.append(start, [int(c_arr[i][0]), int(s_arr[i][0])])
                    end = np.append(end, [int(c_arr[i][1]), int(s_arr[i][1])])
                self.segment_type = "cs"
        else:
            if s_size == c_size + 1:
                for i in range(c_size):
                    start = np.append(start, [int(s_arr[i][0]), int(c_arr[i][0])])
                    end = np.append(end, [int(s_arr[i][1]), int(c_arr[i][1])])
                i += 1
                start = np.append(start, [int(s_arr[i][0])])
                end = np.append(end, [int(s_arr[i][1])])
                self.segment_type = "scs"
            elif c_size == s_size + 1:
                for i in range(s_size):
                    start = np.append(start, [int(c_arr[i][0]), int(s_arr[i][0])])
                    end = np.append(end, [int(c_arr[i][1]), int(s_arr[i][1])])
                i += 1
                start = np.append(start, [int(c_arr[i][0])])
                end = np.append(end, [int(c_arr[i][1])])
                self.segment_type = "csc"
            else:
                sys.exit("straight size is ", s_size, " but corner size is ", c_size)
        self.start = start
        self.end = end

    def check_start_is_straight(self):
        c_arr = self.corners_2d
        s_arr = self.straight_2d
        if int(c_arr[0][0]) < int(s_arr[0][0]):
            return False
        elif int(c_arr[0][0]) > int(s_arr[0][0]):
            return True
        else:
            sys.exit("Corner's first index == Straight's first index")
