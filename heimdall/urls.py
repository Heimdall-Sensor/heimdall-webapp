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
from django.conf.urls import patterns, url

from heimdall import views

urlpatterns = patterns('',
    url(r'camera$', views.camera, name='camera'),
    url(r'calibrate$', views.calibrate, name='calibrate'),
    url(r'detectfloor$', views.detectfloor, name='detectfloor'),
    url(r'toggle$', views.toggle, name='toggle'),
    url(r'set_rate$', views.set_rate, name='set_rate'),
    url(r'source_change$', views.toggle_source, name='toggle_source'),
    url(r'add_mask$', views.add_mask, name='add_mask'),
    url(r'remove_mask$', views.remove_mask, name='remove_mask'),
    url(r'toggle_mask$', views.toggle_mask, name='toggle_mask'),
    url(r'get_mask_list$', views.get_mask_list, name='get_mask_list'),
    url(r'get_status$', views.get_status, name='get_status'),
    url(r'get_movie$', views.get_movie, name='get_movie'),
    url(r'get_movie_thumb$', views.get_movie_thumb, name='get_movie_thumb'),
    url(r'shutdown$', views.shutdown, name='shutdown'),
    url(r'^$', views.index, name='index')
)
