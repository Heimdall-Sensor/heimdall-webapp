#
#  Heimdall Web App manages and interacts with Heimdall Detect Node and Heimdall Image Handler
#  and visualizes detections.
# 
#  Copyright (C) 2017 Christof Oost, Amir Shantia, Ron Snijders, Egbert van der Wal
#
#  This program is free software: you can redistribute it and/or modify
#  it under the terms of the GNU Affero General Public License as
#  published by the Free Software Foundation, either version 3 of the
#  License, or (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU Affero General Public License for more details.
#
#  You should have received a copy of the GNU Affero General Public License
#  along with this program.  If not, see <http://www.gnu.org/licenses/>.
#
import rospy
import numpy
import time
import random

from heimdall_msgs.msg import PolygonArray

class ROSBBoxCallback(object):
    @classmethod
    def get_instance(cls):
        if not hasattr(cls, 'instance') or not cls.instance:
            cls.instance = ROSBBoxCallback()
        return cls.instance

    def __init__(self):
        self.sub_background = rospy.Subscriber("/dd/polygons", PolygonArray, self.bbox_background_callback)
        self.sub_motion = rospy.Subscriber("/dd/polygons_motion", PolygonArray, self.bbox_motion_callback)

        # Initialize polygon storage
        self.polygons_background = []
        self.polygons_background_time = time.time()

        self.polygons_motion = []
        self.polygons_motion_time = time.time()

    def bbox_background_callback(self, msg):
        self.polygons_background = []
        self.polygons_background_time = time.time()
        for poly in msg.polygons:
            np_poly = []
            for point in poly.points:
                np_poly.append([point.x, point.y])
            self.polygons_background.append(numpy.array(np_poly, dtype=numpy.int32))

    def bbox_motion_callback(self, msg):
        self.polygons_motion = []
        self.polygons_motion_time = time.time()
        for poly in msg.polygons:
            np_poly = []
            for point in poly.points:
                np_poly.append([point.x, point.y])
            self.polygons_motion.append(numpy.array(np_poly, dtype=numpy.int32))

