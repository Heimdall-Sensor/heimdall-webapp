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
from sensor_msgs.msg import Image
from std_msgs.msg import String
from heimdall_msgs.srv import *

from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy
import time
import datetime
import random
import json
import copy
import glob
import os

from ROSBBoxCallback import ROSBBoxCallback

IMAGE_TOPIC = [
    #('/camera/rgb/image_rect_color', "bgr8"),
    ('/usb_cam/image_raw', "bgr8"),
    ('/camera/depth/image_raw', "16UC1"),
    #('/camera/depth/image_rect', "32FC1"),
    #('/camera/depth/image_rect_raw', "16UC1")
    #('/camera/depth_registered/hw_registered/image_rect_raw', "16UC1")
]

IMAGE_PATH = "/opt/heimdall/camera/"
IMAGE_ARCHIVE = 60
IMAGE_HR_ARCHIVE = 3

instance_id = 1

RANDOM_SEED = random.random() * 100

class ROSImageCallback(object):
    instances = {}
    next_id = 0

    @classmethod 
    def get_instance(cls):
        global instance_id
        orig_id = instance_id
        if instance_id not in cls.instances:
            instance_id = ROSImageCallback.generate_instance()
            img_cb = ROSImageCallback.instances[instance_id]
            cls.instances[instance_id] = img_cb
            img_cb.set_topic(*IMAGE_TOPIC[0])
            img_cb.touch()
        else:
            img_cb = cls.instances[instance_id]
        return img_cb

    @classmethod
    def generate_instance(cls):
        cur_id = ROSImageCallback.next_id
        instance = ROSImageCallback(cur_id)
        ROSImageCallback.instances[ROSImageCallback.next_id] = instance
        ROSImageCallback.next_id += 1
        instance.log("Generated id with id %d" % cur_id)
        return cur_id

    def __init__(self, id):
        self.id = id
        self.img_count = 0
        self.last_image = None
        self.last_image_time = None
        self.topic = r''
        self.image_type = r''
        self.topic_idx = 0
        self.last_use = time.time()
        self.img_sub = None
        self.bridge = CvBridge()
        self.mask = None
        self.mask_gray = None

        self.notification_subscriber = rospy.Subscriber("/heimdall/notification", String, self.notification_callback)
        self.notifications = []
        self.notification_cleanup = 0

    def cleanup_notifications(self):
        if self.notification_cleanup < time.time() - 5:
            bak = self.notifications
            self.notifications = []
            for t, nt in bak:
                if t > time.time() - 15:
                    self.notifications.append((t, nt))

    def get_notifications(self, since):
        self.cleanup_notifications()
        return [y for x, y in self.notifications if x >= since]

    def notification_callback(self, msg):
        self.log("Notification received: %s" % (repr(msg)))
        try:
            t = time.time()
            now = datetime.datetime.now()
            notification = json.loads(msg.data)
            notification['received'] = t
            self.notifications.append((t, notification))
        except ValueError, e:
            #rospy.logerr("No valid JSON received: %s" % msg)
            self.notifications.append((time.time(), {"message": "Onjuist geformuleerde notificatie ontvangen: \"%s\"" % msg.data, "level": "HIGH", "timeout": 4000, "received": time.time()}))

        # Generate a movie
        ago = now - datetime.timedelta(0, 30)
        vals = now.timetuple()[0:5]
        filename = "notification-%04d%02d%02d%02d%02d" % vals

        self.log("Generating movie file to %s" % filename)

        req = GenerateMovieRequest()
        req.start.data = round(time.time() - 30)
        req.end.data = round(time.time() + 10)
        req.name.data = filename
        if self.last_image_time > 0:
            req.thumbnail.data = round(self.last_image_time)
        else:
            req.thumbnail.data = round(time.time())

        try:
            self.log("Waiting for service")
            rospy.wait_for_service('/heimdall/generate_movie', 1)
            srv = rospy.ServiceProxy('/heimdall/generate_movie', GenerateMovie)
            self.log("Excecuting request")
            srv(req)
        except rospy.ROSException, e:
            self.log("Service not available")
            pass
        
        threshold = now - datetime.timedelta(0, 3 * 86400)
        vals = threshold.timetuple()[0:5]
        threshold = int("%04d%02d%02d%02d%02d" % vals)

        # Check if we should remove some movies
        pos = len(IMAGE_PATH) + 13
        for filename in glob.glob(IMAGE_PATH + "notification-*.mp4"):
            st = int(filename[pos:pos + 12])
            if st < threshold:
                self.log("Removing notification movie %s" % filename)
                os.unlink(filename)

    def next_topic(self):
        global IMAGE_TOPIC
        mx = len(IMAGE_TOPIC)
        self.topic_idx += 1

        if self.topic_idx >= mx:
            self.topic_idx = 0

        self.set_topic(*IMAGE_TOPIC[self.topic_idx])

    def touch(self):
        if not self.img_sub:
            if not self.topic:
                return
            self.last_image = None
            self.img_count = 0
            self.img_sub = rospy.Subscriber(self.topic, Image, self.image_callback)
            self.log("Subsribed to %s -> %s" % (self.topic, repr(self.img_sub)))
        else:
            self.log("Already Subsribed to %s -> %s" % (self.topic, repr(self.img_sub)))
        self.last_use = time.time()
        time.sleep(0.5)

    def set_topic(self, topic, type):
        self.topic = topic
        self.image_type = type
        if self.img_sub:
            self.img_sub.unregister()
            self.img_sub = rospy.Subscriber(topic, Image, self.image_callback)

    def stop_subscriber(self):
        if self.img_sub:
            self.img_sub.unregister()
            self.img_sub = None

    def image_callback(self, image):
        self.log("Image callback called for topic %s" % self.topic);
        try:
            self.last_image = self.bridge.imgmsg_to_cv2(image, self.image_type)
            if self.last_image.dtype == numpy.uint16:
                self.last_image -= 400
                self.last_image >>= 5
                self.last_image = self.last_image.astype(numpy.uint8)
            elif self.last_image.dtype == numpy.complex64:
                self.last_image -= 400
                self.last_image = self.last_image.astype(numpy.uint8)
        except BaseException, e:
            raise

        self.img_count += 1
        bboxes = ROSBBoxCallback.get_instance()

        # Get polygons from ROSBBoxCallback, but only if it was updated in the past 2 seconds
        # otherwise, it's considered outdated.
        polys = bboxes.polygons_background if bboxes.polygons_background_time > (time.time() - 2) else []
        polys2 = bboxes.polygons_motion if bboxes.polygons_motion_time > (time.time() - 2) else []

        # Draw the polygons on the image, using a random color for reach,
        # but using the same seed on each image. This should provide color consistency
        # if the polygons are detected in the same order.
        random.seed(RANDOM_SEED)
        for poly in polys:
            color = (0, 0, 128 + int(round(random.random() * 127)))
            cv2.polylines(self.last_image, [poly], True, color, 2)

        for poly in polys2:
            color = (0, 128 + int(round(random.random() * 127)), 0)
            cv2.polylines(self.last_image, [poly], True, color, 1)

        #if self.mask is not None:
        #    ma = numpy.ma.masked_array(self.last_image, mask=self.mask)
        #    self.last_image = numpy.ma.filled(ma, fill_value=0)

        self.store_image(self.last_image)

        if self.image_type == "bgr8" and self.mask is not None:
            ma = numpy.ma.masked_array(self.last_image, mask=self.mask)
            self.last_image = numpy.ma.filled(ma, fill_value=0)
        elif self.image_type == "16UC1" and self.mask_gray is not None:
            ma = numpy.ma.masked_array(self.last_image, mask=self.mask_gray)
            self.last_image = numpy.ma.filled(ma, fill_value=0)

    def store_image(self, img):
        if not os.path.exists(IMAGE_PATH):
            os.makedirs(IMAGE_PATH)

        self.last_image_time = time.time()
        stamp = datetime.datetime.now()
        vals = stamp.timetuple()[0:6] + (round(stamp.microsecond / 1000), )
        filename = IMAGE_PATH + "frame_%4d%02d%02d%02d%02d%02d%03d.jpg" % vals
        link_file = IMAGE_PATH + "frame_latest.jpg"
        cv2.imwrite(filename, img)

        if os.path.islink(link_file):
            os.unlink(link_file)

        os.symlink(filename, link_file)

        threshold = stamp - datetime.timedelta(0, 60 * IMAGE_ARCHIVE)
        hr_threshold = stamp - datetime.timedelta(0, 60 * IMAGE_HR_ARCHIVE)

        vals = threshold.timetuple()[0:6] + (round(threshold.microsecond / 1000),)
        threshold = int("%4d%02d%02d%02d%02d%02d%03d" % vals)

        vals = hr_threshold.timetuple()[0:6] + (round(hr_threshold.microsecond / 1000),)
        hr_threshold = int("%4d%02d%02d%02d%02d%02d%03d" % vals)

        pos = len(IMAGE_PATH) + 6
        last_sec = None
        for filename in sorted(glob.iglob(IMAGE_PATH + "frame_*.jpg")):
            if filename[pos:pos + 6] == "latest":
                continue
            st = int(filename[pos:pos + 17])
            sec = int(filename[pos + 12:pos + 14])

            if st < threshold:
                #self.log("Removing %s - out of range" % filename)
                os.unlink(filename)
            elif st < hr_threshold:
                if last_sec == sec:
                    #pself.log("Removing %s - out of HR range - %d sec already represented" % (filename, sec))
                    os.unlink(filename)
                last_sec = sec

    def set_mask(self, mask, mask_gray):
        self.mask = mask
        self.mask_gray = mask_gray

    def in_mask(self, poly):
        if self.mask is None:
            return True

        for poly in polys:
            for vertex in poly:
                (x, y) = vertex

    def log(self, msg):
        print("LOG: %s" % msg)
