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
"""
WSGI config for heimdall_webapp project.

It exposes the WSGI callable as a module-level variable named ``application``.

For more information on this file, see
https://docs.djangoproject.com/en/1.6/howto/deployment/wsgi/
"""

import os
import json
import subprocess
import sys
os.environ.setdefault("DJANGO_SETTINGS_MODULE", "heimdall_webapp.settings")

if False:
    # Set up ROS by executing a script that prints out all relevant ROS variables in the proper context
    cur_file = os.path.realpath(__file__)
    cur_path = os.path.dirname(cur_file)
    script = os.path.join(cur_path, "ros_variables.sh")

    script_output = subprocess.check_output(script, shell=True)
    ros_env = json.loads(script_output)

    # Make sure that Django can find the Django webapp itself
    ppath = os.path.dirname(cur_path)
    sys.path.append(ppath)

    # Add all ROS environtmental variables
    for k, v in ros_env.iteritems():
        print >> sys.stderr, "%s = %s" % (k, v)
        if k == "PYTHONPATH":
            paths = v.split(":")
            for path in paths:
                if path not in sys.path:
                    sys.path.append(path)
        else:
            os.environ[k] = v

import rospy
rospy.init_node("heimdall_webapp", disable_signals=True)

from django.core.wsgi import get_wsgi_application
application = get_wsgi_application()
