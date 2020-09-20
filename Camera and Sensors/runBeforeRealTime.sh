#!/bin/bash
# 1. Run this script in terminal before running receive_sim_cam.py to download pip and cv2
# 2. Make sure to "apt-get update" prior to running this script 
# 3. (Don't forget to set the execute permission: "ls -l runBeforeRealTIme.sh")

sudo apt-get install python-pip
pip install opencv-python
python
  import cv2
  exit()
