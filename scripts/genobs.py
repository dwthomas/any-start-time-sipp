#!/usr/bin/env python3

from structs import read_map
import numpy as np
import sys

def make_obs(n, m):
    obs = set()
    xs = np.arange(m.width)
    ys = np.arange(m.height)
    while (len(obs) < n):
        x = np.random.choice(xs)
        y = np.random.choice(ys)
        if m.occupancy[y, x] == 0:
            s = str(x) + " " + str(y) + " 1.0"
            obs.add(s)
    return obs

def print_obs(obs, filename):
    with open(filename, "w") as obsfile:
        print("\n".join(obs), file = obsfile)

if __name__=="__main__":
    mapfile = sys.argv[1]
    n = int(sys.argv[2])
    m = int(sys.argv[3])
    outobsfile = sys.argv[4]
    for i in range(m):
        m = read_map(mapfile)
        obs = make_obs(n, m)
        print_obs(obs, outobsfile)