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
from django.http import HttpResponse, Http404, HttpResponseServerError
from django.shortcuts import render
from django.contrib.auth.decorators import login_required

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool

from heimdall_msgs.srv import *

from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy
import time
import random
import json
import glob
import os
import sys
import random

from modules.Masks import Masks
from modules.Notifications import Notifications
from modules.ImageHandler import ImageHandler

IMAGE_PATH = "/opt/enacer/camera/"

publishers = {}
services = {}

# Create your views here.
@login_required
def index(request):
    context = {}
    return render(request, 'heimdall/index.html', context)

@login_required
def set_rate(request):
    inst = ImageHandler.get_instance();
    rate = float(request.POST['rate'])
    resp = inst.set_rate(rate)

    if resp is None:
        return HttpResponse(json.dumps({'success': False, 'message': 'Failed to set rate'}), content_type='application/json')

    return HttpResponse(json.dumps({'success': True, 'message': 'Rate was set to %f Hz' % rate}), content_type='application/json')

@login_required
def camera(request):
    inst = ImageHandler.get_instance()
    
    if not inst.wait_for_image(10):
        raise Http404('No image is currently available')

    is_uwsgi = sys.argv[0] == "uwsgi"
    
    if is_uwsgi:
        response = HttpResponse()
        response['X-Accel-Redirect'] = '/camera_img/frame_last.jpg'
        response['Content-type'] = "image/jpeg"
        return response

    path = IMAGE_PATH + 'frame_last.jpg'
    print("Servicing image: %s" % path)
    try:
        data = open(path, 'r').read()
    except Exception, e:
        print(repr(e))
        raise Http404('No image is currently available')

    return HttpResponse(data, content_type='image/jpeg')

@login_required
def detectfloor(request):
    resp = execute_command("sgiet nou us op", "/detect_floor")
    msg = resp if resp is not None else "Floor detection service is not available"
    return HttpResponse(json.dumps({'success': resp is not None, 'message': msg}), content_type='application/json')
    
@login_required
def toggle(request):
    new_state = request.POST['status'] == u'true'

    cmd = "detect:on" if new_state else "detect:off"
    resp = execute_command(cmd)

    return HttpResponse(json.dumps({'success': True, 'message': resp}), content_type='application/json')

@login_required
def calibrate(request):
    resp = execute_command("calibrate")
    if resp is None:
        return HttpResponseServerError(json.dumps({'error': 'Could not calibrate'}), content_type='application/json')
    return HttpResponse(json.dumps({'success': True}), content_type='application/json')

@login_required
def toggle_source(request):
    inst = ImageHandler.get_instance()
    inst.toggle_source()

    return HttpResponse(json.dumps({'success': True}), content_type='application/json')

@login_required
def get_mask_list(request):
    m = Masks.get_instance()
    lst = m.get_mask_list()

    if lst == False:
        return HttpResponseServerError(json.dumps({'error': 'Masks could not be retrieved - service is unavailable'}), content_type='application/json')

    ih = ImageHandler.get_instance()
    mask_list = []

    for el in lst:
        mask_list.append({'id': el, 'label': el, 'isChecked': el in ih.enabled_masks})

    return HttpResponse(json.dumps({'success': True, 'masks': mask_list}), content_type='application/json')

@login_required
def toggle_mask(request):
    m = Masks.get_instance()
    label = request.POST['mask'];
    state = request.POST['state'];

    try:
        if state == "true":
            m.enable_mask(label)
        else:
            m.disable_mask(label)
    except Exception, e:
        return HttpResponse(json.dumps({'success': False, 'message': 'Mask does not exist: %s' % str(e)}), content_type='application/json')

    return HttpResponse(json.dumps({'success': True}), content_type='application/json')


@login_required
def add_mask(request):
    m = Masks.get_instance()

    num_points = int(request.POST['num_points'])
    pixels = []

    for p in range(num_points):
        point = request.POST.getlist('path[%d][]' % p)
        point = [float(point[0]), float(point[1])]
        pixels.append(point)

    resp = m.add_mask(request.POST['name'], pixels)

    return HttpResponse(json.dumps({'success': resp == "", 'message': resp}), content_type='application/json')

@login_required
def remove_mask(request):
    m = Masks.get_instance()

    resp = m.remove_mask(request.POST['name'])
    return HttpResponse(json.dumps({'success': resp == "", 'message': resp}), content_type='application/json')

@login_required
def set_active_mask(request):
    m = Masks.get_instance()
    mask = m.get_mask(request.POST['name'])
    m.enable_mask(mask)
    return HttpResponse(json.dumps({'success': resp == "", 'message': resp}), content_type='application/json')

@login_required
def shutdown(request):
    f = open("/tmp/shutdown", "w")
    f.write('yes')
    f.close()

@login_required
def get_movie_thumb(request):
    f = request.GET['filename']
    
    f = f.replace("/", "")
    f = f.replace("..", "")
    f = f.replace("mp4", "jpg")
    f = f.replace("avi", "jpg")

    if f[0:13] != "notification_":
        raise Http404('Thumbnail %s is not available - 1' % f)

    if f[-4:] != ".jpg":
        raise Http404('Thumbnail %s is not available - 2' % f)

    full_name = IMAGE_PATH + f
    print(repr(full_name))

    if not os.path.exists(full_name):
        raise Http404('Thumbnail %s is not available - 3' % f)

    response = HttpResponse()
    response['X-Accel-Redirect'] = '/camera_img/' + f
    response['Content-type'] = 'image/jpeg'
    return response

@login_required
def get_movie(request):
    f = request.GET['filename']
    
    f = os.path.basename(f)
    f = f.replace("/", "")

    if f[0:13] != "notification_":
        raise Http404('Video %s is not available' % f)

    full_name = IMAGE_PATH + f

    if not os.path.exists(full_name):
        raise Http404('Video %s is not available' % f)

    response = HttpResponse()
    response['X-Accel-Redirect'] = '/camera_img/' + f
    if full_name[-3:] == "mp4":
        response['Content-type'] = "video/mp4"
    elif full_name[-3:] == "ogg" or full_name[-3:] == "ogv":
        response['Content-type'] = "video/ogg"
    elif full_name[-4:] == "webm":
        response['Content-type'] = "video/webm"
    elif full_name[-3:] == "m4v":
        response['Content-type'] = "video/m4v"
    elif full_name[-3:] == "avi":
        response['Content-type'] = "video/avi"
    else:
        raise Http404('Video %s is not available' % f)

    return response

@login_required
def get_status(request):
    resp = execute_command('status')
    if not resp:
        return HttpResponseServerError("{'error': 'Status could not be retrieved - service is unavailable'}", content_type='application/json')

    since = 0
    if 'since' in request.POST:
        since = float(request.POST['since'])

    resp = resp.replace("'", '"')
    try:
        resp = json.loads(resp)
    except Exception, e:
        print(repr(e))
        return HttpResponseServerError(json.dumps({'error': 'Status could not be retrieved', 'exception': str(e), 'response': resp}), content_type='application/json')

    if not resp['mode']:
        return HttpResponseServerError(json.dumps({'error': 'Unknown status'}), content_type='application/json')

    ih = ImageHandler.get_instance()
    active_type = ih.active_type

    not_handler = Notifications.get_instance()
    notifications = not_handler.get_notifications(since)

    response = {
        'success': True,
        'mode': resp['mode'],
        'img_type': active_type,
        'since': since,
        'timestamp': time.time(),
        'notifications': notifications
    }

    try:
        m = Masks.get_instance()
        lst = m.get_mask_list()
        mask_list = []

        for el in lst:
            mask_list.append({'id': el, 'label': el, 'isChecked': el in ih.enabled_masks})

        response['masks_available'] = mask_list
        response['masks'] = ih.enabled_masks
    except Exception, e:
        print(repr(e))

    try:
        movie_files = glob.glob(IMAGE_PATH + "notification_*.mp4")
        print(repr(movie_files))
        movies = []
        for f in movie_files:
            movies.append(os.path.basename(f))
        response['movies'] = [x for x in reversed(sorted(movies))]
    except:
        print(repr(e))

    return HttpResponse(json.dumps(response), content_type='application/json')

def get_publisher(topic, message_type):
    global publishers
    if topic not in publishers:
        publishers[topic] = rospy.Publisher(topic, message_type)

    return publishers[topic]

def execute_command(cmd, node = "/dd/command"):
    global services
    n = "command_%s" % node
    if n not in services:
        try:
            rospy.wait_for_service(node, timeout=2)
        except rospy.ROSException:
            return None
        services[n] = rospy.ServiceProxy(node, CommandSrv)

    srv = services[n]
    resp = srv(cmd)
    return resp.result

# 2017-04-20 - Doesnt seem to be used?!
# def publish_command(command):
#     global publishers
#     svc = '/dd/command'
#     
#     if svc not in publishers:
#         rospy.wait_for_service(svc)
#         publishers[svc] = rospy.ServiceProxy(svc, HeimdallCommand)
# 
#     response = None
#     try:
#         response = publishers[svc](command)
#         return json.loads(response)
#     except rospy.ServiceException as e:
#         return {'error': 'Service exception: %s' % str(e)}
#     except ValueError as e:
#         return {'error': 'JSON parsing exception while parsing', 'response': response}
