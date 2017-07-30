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
from std_msgs.msg import String
from heimdall_msgs.srv import *

from ImageHandler import ImageHandler

import time
import json
import datetime

NOTIFICATION_TOPIC = "/heimdall/notification"

class Notifications(object):
    instance = None

    @classmethod 
    def get_instance(cls):
        if not cls.instance:
            cls.instance = Notifications()
        return cls.instance

    def __init__(self):
        print("Creating Notifications instance")
        self.notification_subscriber = rospy.Subscriber(NOTIFICATION_TOPIC, String, self.notification_callback)
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
            self.notifications.append((time.time(), {"message": "Incorrectly formed notification received: \"%s\"" % msg.data, "level": "HIGH", "timeout": 4000, "received": time.time()}))

        self.log("Generating video")
        ih = ImageHandler.get_instance()
        ih.generate_video()

    def log(self, msg):
        print("LOG: %s" % msg)
