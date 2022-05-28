#!/usr/bin/env python3
import cv2
import numpy as np
import matplotlib.pyplot as plt
from SystemValues import *
class MapCreator:

    def create_map(self, width, height, resolution, polygons):
        image = np.zeros((width,height,3), np.uint8)

        for polygon in polygons:
            npa = np.array(polygon)
            cv2.fillPoly(
                image,
                [npa],
                color=(255,255,255),
            )
        #cv2.cv
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        cv2.imwrite(SystemValues.dataPath + SystemValues.greenZoneMapName, gray)

        plt.imshow(image)
        plt.show()

if __name__ == '__main__':

    polygons = [[[400, 400], [800,400], [800,800], [400,800]],
                        [[1000, 1000], [1100, 1000], [1200, 900], [800, 800], [1400, 900]]]


    MapCreator().create_map(4000,4000,0.05, polygons)
