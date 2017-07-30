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
import json
import time
import math

from std_msgs.msg import String
from heimdall_msgs.srv import *

COMMAND_SERVICE = "/heimdall/image_handler"
UPDATE_TOPIC = "/heimdall/image_updated"

class ImageHandler(object):
    instance = None

    @classmethod 
    def get_instance(cls):
        if not cls.instance:
            cls.instance = ImageHandler()
        return cls.instance

    def __init__(self):
        print("Creating ImageHandler instance")
        self.srv = None
        self.active_type = None
        self.enabled_masks = []
        self.refresh = 250
        self.last_image = None

    def wait_for_image(self, timeout=2):
        if self.last_image:
            nw = time.time()
            diff = int((nw - self.last_image) * 1000)

            if diff < int(0.5 * self.refresh):
                return True
            else:
                print("Last image was received %d ms ago - too long for to of %d" % (diff, self.refresh))
        try:
            msg = rospy.wait_for_message(UPDATE_TOPIC, String, timeout=timeout)
            data = json.loads(msg.data)
            self.active_type = data['type']
            self.enabled_masks = data['enabled_masks']
            self.refresh = int(data['interval'])
            self.last_image = time.time()
            return True
        except rospy.ROSException, e:
            return False

    def get_srv(self):
        if not self.srv:
            try:
                rospy.wait_for_service(COMMAND_SERVICE, timeout=2)
            except rospy.ROSException:
                return None

            self.srv = rospy.ServiceProxy(COMMAND_SERVICE, CommandSrv)
        return self.srv

    def toggle_source(self):
        if self.active_type == "depth":
            return self.switch_to_rgb()
        return self.switch_to_depth()

    def execute_command(self, cmd):
        srv = self.get_srv()
        if not srv:
            print("Not active service")
            raise Exception("No service")

        if not isinstance(cmd, basestring):
            cmd = json.dumps(cmd)

        print("Calling service with command %s" % cmd)
        resp = srv(cmd)
        print("Result: %s" % resp.result)
        return resp.result


    def switch_to_rgb(self):
        self.execute_command({'command': 'switch_rgb', 'lock_duration': 30})

    def switch_to_depth(self):
        self.execute_command({'command': 'switch_depth', 'lock_duration': 30})

    def enable_mask(self, mask_name):
        self.execute_command({'command': 'enable_mask', 'label': mask_name})

    def disable_mask(self, mask_name):
        self.execute_command({'command': 'disable_mask', 'label': mask_name})

    def get_active_masks(self):
        resp = self.execute_command({'command': 'get_enabled_masks'})
        masks = json.loads(resp)
        return masks['enabled_masks']

    def set_rate(self, rate):
        return self.execute_command({'command': 'set_rate', 'rate': rate, 'lock_duration': 180})

    def generate_video(self):
        end = int(time.time() + 5)
        start = end - 7
        thumbnail = end - 2

        return self.execute_command({'command': 'generate_video', 'start': start, 'end': end, 'thumbnail': thumbnail})
