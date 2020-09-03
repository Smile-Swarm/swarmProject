#!/usr/bin/env python
import time
from rosthrottle import MessageThrottle

if __name__=='__main__':
    intopic = '/leader_pose'
    outtopic = '/leader_pose_throttled'
    rate = 5.0
    t = MessageThrottle(intopic, outtopic, rate)
    pid = t.start()
