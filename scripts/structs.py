import numpy as np
import h5py
from collections import deque
import sys

def shift(s_x, s_y, d_x, d_y):
    return int(d_x != s_x) | (int(d_y != s_y) << 1)

def domove(x, y, m):
    return x + (m & 0x01), y + ((m & 0x02) >> 1)
        
a2o = {
    ".":False,
    "G":False,
    "S": False,
    "@": True,
    "O": True,
    "T": True,
    "W": True
}

def read_map(filename):
    with open(filename) as f:
        #header 
        f.readline()
        height = int(f.readline().strip().split()[1])
        width = int(f.readline().strip().split()[1])
        f.readline()
        occupancy = np.zeros((width, height))
        for j in range(height):
            line = f.readline()
            for i in (range(width)):
                occupancy[i,j] = a2o[line[i]]
    return Map(occupancy)


class Map:
    def __init__(self, occupancy):
        self.occupancy = occupancy
        self.width = occupancy.shape[1]
        self.height = occupancy.shape[0]

class Path:
    def __init__(self, locs, times, agent_speed):
        self.locs = locs
        self.times = times
        self.agent_speed = agent_speed
    
    def occupies(self, time, safe_intervals):
        for i in range(len(self.times)):
            if self.times[i] >= time:
                break
        i -= 1
        if i >= len(self.times)-1:
            return [(self.locs[-1, 0], self.locs[-1, 1], 0)]
        s_x = self.locs[i, 0]
        s_y = self.locs[i, 1]
        d_x = self.locs[i+1, 0]
        d_y = self.locs[i+1, 1]
        move = shift(s_x, s_y, d_x, d_y)
        retval = []
        #retval = [(s_x, s_y, move)]
        if move > 0:
            retval.append((min(s_x, d_x), min(s_y, d_y), move))
        if abs(self.times[i] - time) < abs(self.times[i+1] - time):
            retval.append((s_x, s_y, 0))
        else:
            retval.append((d_x, d_y, 0))

        for val in retval:
            if not safe_intervals.is_safe(*val, time, debug = True):
                print("UNSAFE ACTION")
                print(d_x, d_y, time)
                #sys.exit()        
        return retval

class SafeIntervals:
    def __init__(self, locs, times, map):
        self.intervals = np.empty((map.width, map.height, 4),  dtype = object)
        for i in range(self.intervals.shape[0]):
            for j in range(self.intervals.shape[1]):
                for k in range(self.intervals.shape[2]):
                    self.intervals[i, j, k] = deque()
        for i in range(len(locs)):
            self.intervals[locs[i, 0], locs[i, 1], locs[i, 2]].append(times[i])
        print(self.intervals.shape)

    def is_safe(self, x, y, move, time, debug = False):
        tp = []
        if (x < 0) or (y < 0) or (x >= self.intervals.shape[0]) or (y >= self.intervals.shape[1]):
            return False 
        for interval in self.intervals[x, y, move]:
            tp.append((x, y, interval))
            if (interval[0] <= time) and (interval[1] >= time):
                return True
        if debug and (len(tp) > 0):
            print(tp)
        return False

class Results:
    def __init__(self, resfile):
        with h5py.File(resfile, 'r') as hdf5_file:
            self.map = Map(np.asarray(hdf5_file["map"]))
            path_loc = np.asarray(hdf5_file["path_locs"])
            path_times = np.asarray(hdf5_file["path_times"])
            agent_speed = float(np.asarray(hdf5_file["agent_speed"]))
            self.path = Path(path_loc, path_times, agent_speed)
            try:
                sil = np.asarray(hdf5_file["safe_intervals_locs"])
                sit = np.asarray(hdf5_file["safe_intervals_times"])
                self.safe_intervals = SafeIntervals(sil, sit, self.map)
            except:
                self.safe_intervals = SafeIntervals([], [], self.map)
            self.fate = str(np.asarray(hdf5_file["agent_fate"]))
            self.goal_loc = np.asarray(hdf5_file["goal_location"])