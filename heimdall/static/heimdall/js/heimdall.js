//
//  Heimdall Web App manages and interacts with Heimdall Detect Node and Heimdall Image Handler
//  and visualizes detections.
// 
//  Copyright (C) 2017 Christof Oost, Amir Shantia, Ron Snijders, Egbert van der Wal
//
//  This program is free software: you can redistribute it and/or modify
//  it under the terms of the GNU Affero General Public License as
//  published by the Free Software Foundation, either version 3 of the
//  License, or (at your option) any later version.
//
//  This program is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU Affero General Public License for more details.
//
//  You should have received a copy of the GNU Affero General Public License
//  along with this program.  If not, see <http://www.gnu.org/licenses/>.
//
// // Dutch 
// var L = {
//     'AREAS': "Gebieden",
//     'AREA_NAME': "Geef het gebied een naam: ",
//     'AREA_DUPLICATE': "Het gebied '%s' is al gedefiniëerd. Geef een andere naam op!",
//     'WAITING_CONNECTION': "Wachten op verbinding met Heimdall..."
// };

// English
var L = {
    'MOVIES': 'Movies',
    'AREAS': "Areas",
    'AREA_NAME': "Please provide a name for the selected area: ",
    'AREA_DUPLICATE': "The area '%s' has already been defined. Please provide a different name.",
    'AREA_ADDED': "Area '%s' was added",
    'AREA_REMOVED': "Area '%s' was removed",
    'WAITING_CONNECTION': "Awaiting connection to Heimdall...",
    'MASK_FETCH_ERROR': 'Failed to obtain mask list - retrying'
};

$(function() {
    // Show loading spinner
    showWaitingOverlay();

    $.ajaxSetup({
        'timeout': 15000
    });

    // Request regular status updates
    window.status_request = null;
    window.last_status = 0;
    //window.status_interval = setInterval(requestStatus, 60000);
    requestStatus();

    // Set up the camera refresh
    var camera = $('#camera')
    var hz = parseInt(camera.data('refresh'));
    var refresh = 1000 / hz;
    var plain_url = camera.attr('src');

    var interval = setInterval(refreshCamera, 10000); // Fallback refresh
    setTimeout(refreshCamera, Math.max(100, refresh - 75));
    camera.data({'basic_url': plain_url, 'interval': interval, 'refresh': refresh});
    
    // Make sure the appropriate button is enabled
    $('#refresh-rate input').each(function () {
        var val = $(this).val();
        if (val == hz)
        {
            $(this).parent().click();
            return false;
        }
    });

    // Toggle detection state
    $('#detect-btn').on('click', function () {
        var btn = $(this);
        var new_state = !$(this).hasClass('enabled');
        $.post('/toggle', {'status': new_state});
        setDetectButtonEnabled(new_state);
    });

    // Shutdown button
    $('#shutdown-btn').on('click', function () {
        $.post('/shutdown');
        var camera = $('#camera');
        camera.data('refresh', 60000);
    });

    // Start calibration
    $('#calibration-btn').on('click', function() { 
        var btn = $(this);
        btn.prop('disabled', true);
        var state = $('button#detect-btn').hasClass('enabled');
        if (state)
        {
            setDetectButtonEnabled(false);
        }

        $.post('/calibrate').always(function () {
            btn.prop('disabled', false);
            setDetectButtonEnabled(true);
        });
    });

    // Detect floor
    $('#floordetect-btn').on('click', function() { 
        var fd_btn = $(this);
        fd_btn.prop('disabled', true);
        $.post('/detectfloor').always(function (resp) {
            if (!resp.success)
                handleNotification({'message': resp.message, 'type': 'danger', "timeout": 5000});

            fd_btn.prop('disabled', false);
            requestMaskListUpdate();
        });
    });


    // Change refresh rate
    $('#refresh-rate label').on('click', function () {
        var hz = $(this).children('input').val();
        var refresh = 1000 / hz;

        var camera = $('#camera');
        camera.data('refresh', refresh);

        window.camera_image_requested = null;
        window.camera_image_stamp = null;
        refreshCamera();

        $.post('/set_rate', {'rate': hz});
    });

    $('#movie-btn').on('click', function () {
        var overlay = $('<div></div>').addClass('overlay clips');
        $('body').append(overlay);

        var movies = $('#movie_list');

        var list = $('<div></div>').attr('id', 'movie_window');
        $('body').append(list);
        list.html(movies.html());
        list.fadeIn();   

        // Create a destroy button
        var btn = $('<button class="btn btn-primary close-button"><i class="glyphicon glyphicon-remove-sign"></i></button>');
        list.append(btn);

        btn.on('click', function () {
            $(this).closest('#movie_window').fadeOut(function () {$(this).remove();});
            $('.overlay.clips').remove();
        });

        list.on('click', 'a,li', function (e) {
            e.preventDefault();

            var $this = $(this);
            if ($this.is('li'))
                $this = $this.find('a');

            var img = $this.closest('li').find('img').attr('src');

            var href = $this.attr('href');
            var video = $('<video></video>').attr('controls', 'controls').attr('poster', img);

            var ratio = 320 / 240;
            var aw = list.innerWidth() * 0.95;
            var ah = list.innerHeight() * 0.95;

            var sw = aw;
            var sh = sw / ratio;
            var l = 0;
            var t = (ah - sh) / 2;

            if (sh > ah)
            {
                sh = ah;
                sw = sh * ratio;
                t = 0;
                l = (aw - sw) / 2;
            }

            video.css({'width': sw, 'height': sh, 'top': t, 'left': l});

            //var src = $('<source />').attr('src', href).attr('type', 'video/avi');
            var src = $('<source />').attr('src', href).attr('type', 'video/mp4');
            //var src2 = $('<source />').attr('src', href.replace("m4v", "webm")).attr('type', 'video/webm');
            //var src3 = $('<source />').attr('src', href.replace("m4v", "ogv")).attr('type', 'video/ogg');

            //video.append(src, src2, src3);
            video.append(src);

            list.html('').append(video);
        });

        $('.overlay.clips').on('click', function (e) {
            $('body .overlay.clips').remove();
            $('#movie_window').remove();
        });

        $(window).on('keyup', function (e) {
            if (e.key == 'Escape')
            {
                list.remove();
                $('body .overlay.clips').remove();
            }
        });
    });

    $('#mask-btn').on('click', function () {
        var btn = $(this);
        var icon = btn.find('i');
        if (icon.hasClass('glyphicon-flag'))
        {
            // Currently recording
            icon.removeClass('glyphicon-flag').addClass('glyphicon-record');
            $('#camera').css('cursor', 'auto');

            var name = prompt(L['AREA_NAME']);
            if (name)
            {
                var param = {'path': window.mask_path_rel, 'name': name, 'num_points': window.mask_path_rel.length};

                var handle_add_mask_response = function (response) {
                    if (!response.success)
                    {
                        if (response.message == "duplicate")
                        {
                            var tmsg = L['AREA_DUPLICATE'];
                            tmsg = tmsg.replace("%s", param['name']);
                            param['name'] = prompt(tmsg);
                            if (!param['name'])
                                return;

                            $.post('/add_mask', param, handle_add_mask_response);
                        }
                        else if (response.message)
                        {
                            handleNotification({'message': msg, 'type': "danger"});
                        }
                    }
                    else
                    {
                        var tmsg = L['AREA_ADDED'];
                        tmsg = tmsg.replace("%s", name);
                        handleNotification({'message': tmsg, 'type': 'success', "timeout": 5000});
                        requestMaskListUpdate();
                    }
                };

                $.post('/add_mask', param, handle_add_mask_response);
            }

            delete window.mask_path_rel;
            delete window.mask_path;
            delete window.mask_svg;
            window.canvas.remove();
            delete window.canvas;
        }
        else
        {
            window.mask_path_rel = [];
            window.mask_path = [];
            
            var img = $('#camera');
            var pos = img.position();
            var width = img.outerWidth();
            var height = img.outerHeight();

            var left = 0;
            var top = 0;
            var el = img;
            var num = 0;
            while (true)
            {
                var p = el.position();
                left += p.left;
                top += p.top;
                if (el.offsetParent()[0] == el[0])
                    break;

                el = el.offsetParent();
                ++num;
                if (num > 10)
                    break;
            }

            window.canvas = Raphael(left, top, width, height);
            window.mask_svg = $(window.canvas.canvas);

            icon.addClass('glyphicon-flag').removeClass('glyphicon-record');
            window.mask_svg.css('cursor', 'crosshair');
            window.mask_svg.on('click', addPoint);
            window.mask_svg.on('dblclick', function () { $('#mask-btn').click(); });
        }
    });

    $('#camera').on('dblclick', function (e) {
        $('#mask-btn').click();
    });

    $('#source-btn').on('click', function () {
        $.post('/source_change');
    });

    $("#mask_list").dropdownCheckbox({
        data: [],
        title: L['AREAS'],
        btnClass: 'btn btn-primary'
    });

    requestMaskListUpdate();
    setInterval(requestMaskListUpdate, 30000);

    $("#mask_list").on('change', 'input[type=checkbox]', function () {
        var el = $(this);
        $.post('/toggle_mask', {mask: el.next('label').text(), state: el.is(':checked')});
    });

    // Scale image to viewport size
    $(window).on('resize', function () {
        var w = $(window).innerWidth();
        var h = $(window).innerHeight();

        var rel_max_width = 0.95;
        var rel_max_height = 0.8;
        var aspect_ratio = 640.0 / 480.0;

        var cam = $('#camera');
        var cw = cam.innerWidth();
        var ch = cam.innerHeight();

        var nw = rel_max_width * w;
        var nh = nw / aspect_ratio;
        if (nh > rel_max_height * h)
        {
            nh = rel_max_height * h;
            nw = nh * aspect_ratio;
        }

        cam.css({'width': nw, 'height': nh});
    }).resize();
});

function requestStatus()
{
    var now = new Date().getTime();
    var tgt = window.status_request_target;
    if (now < tgt)
        return;
    
    if (window.status_request)
    {
        var now = new Date().getTime();
        var elapsed = window.status_request_start - now;
        if (elapsed > 10000) // Don't give up too soon
        {
            // Abort the request
            window.status_request.abort();

            // The abort will trigger the error handler, so it will be retried
        }
        return;
    }

    var streq;
    window.status_request = streq = $.post('/get_status', {'since': last_status}, updateStatus, "json");
    window.status_request_start = new Date().getTime();

    window.status_request.error(function (resp) {
        showWaitingOverlay();
    }).always(function (resp) {
        if (streq === window.status_request)
        {
            setTimeout(requestStatus, 5000);
            window.status_request_target = window.status_request.start + 5000;
            delete window.status_request;
            delete window.status_request_start;
        }
    });
}

function updateStatus(resp)
{
    removeOverlay();
    if (resp.mode)
    {
        setDetectButtonEnabled(resp.mode == "detect");
        if (resp.mode == "calibrate")
            $('button#calibration-btn').prop('disabled', true);
        else
            $('button#calibration-btn').prop('disabled', false);

        if (resp.notifications)
        {
            for (var k in resp.notifications)
                handleNotification(resp.notifications[k]);
        }
        window.last_status = resp.timestamp;
    }

    if (resp.masks_available)
        updateMaskList({'masks': resp.masks_available});

    if (resp.movies)
    {
        var lst = $('#movie_list');
        lst.html('<ul></ul>');
        var txt;
        var stmp;
        var y, m, d, h, m, s;
        var dstr;

        for (var k in resp.movies)
        {
            txt = resp.movies[k];
            stmp = txt.substring(13);

            y = stmp.substring(0, 4);
            m = stmp.substring(4, 6);
            d = stmp.substring(6, 8);
            h = stmp.substring(8, 10);
            m = stmp.substring(10, 12);
            if (stmp.length == 21)
                s = stmp.substring(12, 14);

            dstr = h + ":" + m; // + ":" + s;
    
            //var hr = "https://cloud.robovalley.com:5555/heimdall/" + resp.movies[k];
            var hr = "/get_movie?filename=" + resp.movies[k];
            //var anchor = $('<a></a>').attr('href', '/static/media/' + resp.movies[k]).text(dstr);
            var anchor = $('<a></a>').attr('href', hr).text(dstr);
            var img = $('<img />').attr('src', '/get_movie_thumb?filename=' + resp.movies[k]).attr('title', 'Thumbnail ' + dstr).attr('alt', dstr);
            anchor.append(img);
            var li = $('<li></li>').append(anchor);
            li.on('click', function () { $(this).find('a').click(); });
            lst.append(li); 
        }
    }
}

function addPoint(e)
{
    var img = $(this);
    var btn = $('#mask-btn');
    var icon = btn.find('i');

    var width = img.outerWidth();
    var height = img.outerHeight();

    var posX = e.offsetX;
    var posY = e.offsetY;

    if (e.target.nodeName != "svg")
    {
        var bb = e.target.getBBox();
        posX += bb.x;
        posY += bb.y;
    }

    var relX = posX / width;
    var relY = posY / height;
    window.mask_path_rel.push([relX, relY]);
    window.mask_path.push([posX, posY]);

    var nc = window.canvas.circle(posX, posY, 3);
    nc.attr('fill', '#000').attr('stroke', '#fff');

    if (window.mask_path.length > 1)
    {
        var cur = window.mask_path.length - 1;
        var prev = cur - 1;

        var curPt = window.mask_path[cur];
        var prevPt = window.mask_path[prev];

        var pathStr = "M" + prevPt[0] + "," + prevPt[1] + "L" + curPt[0] + "," + curPt[1];
        var path = window.canvas.path(pathStr).attr({'stroke': '#fff', 'stroke-width': 2});
    }
}


function handleNotification(notification)
{
    var type = 'info';
    if (notification.type)
    {
        type = notification.type;
    }
    else
    {
        if (notification.level > 2 || notification.level == "HIGH")
            type = 'danger';
    }

    var to = notification.timeout ? notification.timeout : 30000;

    var st;
    if (notification.received)
        st = new Date(Math.round(parseFloat(notification.received) * 1000));
    else
        st = new Date();

    var h = st.getHours();
    var m = st.getMinutes();
    if (m < 10)
        m = "0" + m;
    var s = st.getSeconds();
    if (s < 10)
        s = "0" + s;

    var msg = "[" + h + ":" + m + ":" + s + "] " + notification.message;
        
    $.bootstrapGrowl(msg, {
        ele: 'body',
        type: type,
        offset: {from: 'top', amount: 70},
        align: 'right',
        delay: to,
        stackup_spacing: 10
    });
}

function setDetectButtonEnabled(enabled)
{
    if (enabled)
    {
        $('button#detect-btn')
            .addClass('enabled')
            .children('i')
            .addClass('glyphicon-stop')
            .removeClass('glyphicon-play');
    }
    else
    {
        $('button#detect-btn')
            .removeClass('enabled')
            .children('i')
            .removeClass('glyphicon-stop')
            .addClass('glyphicon-play');
    }
}

function refreshCamera()
{
    var now_stamp = (new Date).getTime();

    if (window.camera_image_target > now_stamp)
        return;
    delete window.camera_image_timeout;

    var camera = $('#camera');
    var prev_stamp = window.camera_image_stamp;

    if (window.camera_image_requested)
    {
        if (now_stamp - prev_stamp < 10000)
            return;
    }

    var url = camera.data('basic_url') + "?stamp=" + now_stamp;
    var img = new Image();
    img.src = url;
    img.onload = function () {
        var stamp = window.camera_image_stamp;
        var url = this.src;
        var match = url.match(/stamp=([0-9]+)/);
        var url_stamp = parseInt(match[1]);

        if (url_stamp === stamp)
        {
            delete window.camera_image_requested;
            delete window.camera_image_stamp;

            camera.attr('src', this.src);
            var ref = camera.data('refresh');
            var to = ref - 75;
            window.camera_image_target = new Date().getTime() + to;
            setTimeout(refreshCamera, Math.max(100, to));
        }
        else
            console.log('Received data with incorrect stamp: ' + url);
    };

    window.camera_image_requested = img;
    window.camera_image_stamp = now_stamp;
}

function updateMaskList(response)
{
    console.log(response);
    var lst = $("#mask_list");

    lst.dropdownCheckbox("reset", response.masks);

    $('#mask_list input[type=checkbox]').each(function () {
        var $this = $(this)
        var container = $this.parent();
        var wrapper = $('<div></div>').addClass('content');
        var els = container.children('label,input');
        wrapper.append(els);
        container.append(wrapper);
        var trash = $('<i></i>').addClass('glyphicon glyphicon-trash');
        var link = $('<a href="#" style="display:inline-block;float:right;"></a>').append(trash);

        link.on('click', function () {
            var label = $(this).closest('div.layout').text();
            $.post('/remove_mask', {'name': label}, function (response) {
                if (response.success)
                {
                    requestMaskListUpdate();
                    var tmsg = L['AREA_REMOVED'];
                    tmsg = tmsg.replace("%s", label);
                    handleNotification({'message': tmsg, 'type': 'success', 'timeout': 5000});
                }
                else if (response.message)
                {
                    handleNotification({'message': msg, 'type': "danger"});
                }
            });
        });

        container.prepend(link);
    });
}

function removeOverlay()
{
    $('body').find('.waiting-overlay,.waiting-alert').remove();
}

function showWaitingOverlay()
{
    if ($('body .waiting-alert').length > 0)
        return;

    var w = $(window);

    var overlay = $('<div></div>').addClass('waiting-overlay overlay');

    $('body').append(overlay);

    var notice = $('<div></div>')
        .addClass('alert alert-warning waiting-alert')
        .css({'background-image': 'url(/static/heimdall/img/spinner.gif)', 'background-size': 'auto auto', 'background-repeat': 'no-repeat', 'background-position': '50% 50%'})
        .text(L['WAITING_CONNECTION']);

    $('body').append(notice);
}

function requestMaskListUpdate()
{
    if (window.mask_request)
    {
        var req = window.mask_request;
        var start = window.mask_request_start;
        var now = new Date().getTime();

        if (now - start > 30000)
        {
            // Abort
            req.abort();
            setTimeout(requestMaskListUpdate, 2000);
        }
        return;
    }

    var req = $.post('/get_mask_list')
        .success(updateMaskList)
        .error(function (resp) {
            var tmsg = L['MASK_FETCH_ERROR'];
            handleNotification({'message': tmsg, 'type': 'danger', 'timeout': 500});
        })
        .always(function (resp) {
            delete window.mask_request;
            delete window.mask_request_start;
        });

    window.mask_request = req;
    window.mask_request_start = new Date().getTime();
}
