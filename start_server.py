#!/usr/bin/env python
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
import os, sys

# Set up ROS
import roslib
import rospy
rospy.init_node('heimdall_webapp', disable_signals=True)

import heimdall_webapp.settings

ip = "0.0.0.0"

sys.argv = [sys.argv[0], "runserver", "%s:8000" % ip]

if __name__ == "__main__":
    try:
        os.environ.setdefault("DJANGO_SETTINGS_MODULE", "heimdall_webapp.settings")
        from django.core.management import execute_from_command_line
        execute_from_command_line(sys.argv)
    except Exception, e:
        #rospy.signal_shutdown(e.reason)
        print repr(e)
