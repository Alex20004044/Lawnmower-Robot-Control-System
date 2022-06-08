#!/usr/bin/env python3
import cv2
import numpy as np

def Calculate():
    mowZone = cv2.imread("/home/alex20004044/Diploma/Lawnmower-Robot-Control-System/Data/MowZoneMap.pgm")
    mowZone = cv2.cvtColor(mowZone, cv2.COLOR_BGR2GRAY)
    coverageZone = cv2.imread("/home/alex20004044/Diploma/Lawnmower-Robot-Control-System/Data/CoverageImage.png")
    coverageZone = cv2.cvtColor(coverageZone, cv2.COLOR_BGR2GRAY)
    print(mowZone.dtype)
    print(coverageZone.dtype)
    mowZoneSize = np.count_nonzero(mowZone)

    coverageSize = np.count_nonzero(np.logical_and(mowZone > 0, coverageZone > 0))

    print("MowZoneSize: " + str(mowZoneSize))
    print("CoverageSize: " + str(coverageSize))
    print("Percentage: " + str(coverageSize/mowZoneSize))

if __name__ == '__main__':
    Calculate()