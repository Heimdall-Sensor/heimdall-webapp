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
from heimdall_msgs.srv import *

import ImageHandler
import numpy as np
import cv2

from ImageHandler import ImageHandler

ADD_MASK_SERVICE = "/dd/add_mask"
GET_MASK_LIST_SERVICE = "/dd/get_mask_list"
REMOVE_MASK_LIST_SERVICE = "/dd/remove_mask_list"
COMMAND_SERVICE = "/dd/command"

class Masks(object):
    instance = None

    @classmethod
    def get_instance(cls):
        if not cls.instance:
            cls.instance = Masks()
        return cls.instance

    def __init__(self):
        print("Creating Masks instance")
        self.command_service = None
        self.add_mask_service = None
        self.get_mask_list_service = None
        self.remove_mask_list_service = None

    def execute_command(self, cmd):
        if not self.command_service:
            try:
                rospy.wait_for_service(COMMAND_SERVICE, timeout=2)
            except rospy.ROSException:
                return None
            self.command_service = rospy.ServiceProxy(COMMAND_SERVICE, CommandSrv)

        try:
            srv = self.command_service
            resp = srv(cmd)
            return resp.result
        except:
            self.command_service = None
            raise Exception("Exception while sending command")

    def enable_mask(self, mask):
        inst = ImageHandler.get_instance()
        inst.enable_mask(mask)

    def disable_mask(self, mask):
        inst = ImageHandler.get_instance()
        inst.disable_mask(mask)

    def get_mask_list(self):
        if not self.get_mask_list_service:
            try:
                rospy.wait_for_service(GET_MASK_LIST_SERVICE, timeout=5)
            except rospy.ROSException, e:
                return False

            self.get_mask_list_service = rospy.ServiceProxy(GET_MASK_LIST_SERVICE, GetMaskListSrv)

        try:
            resp = self.get_mask_list_service()
        except:
            print("ERROR: Failed to get mask list, forcing to recreate service...")
            self.get_mask_list_service = None
            return False

        return resp.labels

    def add_mask(self, name, vertices):
        if not self.add_mask_service:
            try:
                rospy.wait_for_service(ADD_MASK_SERVICE, 2)
                self.add_mask_service = rospy.ServiceProxy(ADD_MASK_SERVICE, AddMaskSrv)
            except rospy.ROSException, e:
                print("Add_mask service is not available")
                return False

        # Fixed?!
        w = 640
        h = 480

        for i in range(len(vertices)):
            vertices[i][0] = round(vertices[i][0] * w)
            vertices[i][1] = round(vertices[i][1] * h)

        mask = np.zeros([h, w], dtype='uint8')
        vertices = np.array(vertices, dtype='int32')
        cv2.fillPoly(mask, [vertices], 255)

        req = AddMaskSrvRequest()
        req.label = name
        req.width = mask.shape[1]
        req.height = mask.shape[0]
        req.pixels = range(0, req.width * req.height)
        pos = 0
        for row in mask:
            for col in row:
                req.pixels[pos] = 1 if col > 0 else 0
                pos += 1

        try:
            resp = self.add_mask_service(req)
        except rospy.ServiceException, e:
            return "duplicate"
        except rospy.ROSException, e:
            return "disconnected"

        return resp.result

    def remove_mask(self, name):
        if not self.remove_mask_list_service:
            try:
                rospy.wait_for_service(REMOVE_MASK_LIST_SERVICE, 2)
                self.remove_mask_list_service = rospy.ServiceProxy(REMOVE_MASK_LIST_SERVICE, RemoveMaskListSrv)
            except rospy.ROSException, e:
                return False

        resp = self.remove_mask_list_service([name])
        return resp.result
