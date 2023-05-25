#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# import necessary argumnets 
from turtle import pos
import gi
import cv2
import argparse
import numpy as np
import rospy

from qcar.q_essential import Camera2D

# import required library like Gstreamer and GstreamerRtspServer
gi.require_version('Gst', '1.0')
gi.require_version('GstRtspServer', '1.0')
from gi.repository import Gst, GstRtspServer, GObject
from geometry_msgs.msg import PoseStamped

# Sensor Factory class which inherits the GstRtspServer base class and add
# properties to it.

class RosSubscriber():
    def __init__(self) :
        #subscribe to rostopic to get camera position 
        self.ctrl_sub = rospy.Subscriber("/human_remote/camera_pos", PoseStamped, self.ctrl_callback, queue_size=10)   
        self.position=[1,1,1,1]  
        rospy.init_node('camera_pos')

    def ctrl_callback(self, data):
        # self.position=data
        position=[]
        position.append(data.pose.orientation.x)  # front
        position.append(data.pose.orientation.y)  # rear
        position.append(data.pose.orientation.z)  # left
        position.append(data.pose.orientation.w)  # right
        self.position=position

    def get_position(self):
        return self.position


class SensorFactory(GstRtspServer.RTSPMediaFactory):
    def __init__(self, **properties):
        super(SensorFactory, self).__init__(**properties)
        # self.cap = cv2.VideoCapture(opt.device_id)
        self.fps= opt.fps
        self.number_frames = 0
        self.duration = 1 / self.fps * Gst.SECOND  # duration of a frame in nanoseconds
        self.launch_string = 'appsrc name=source is-live=true block=true format=GST_FORMAT_TIME ' \
                             'caps=video/x-raw,format=BGR,width={},height={},framerate={}/1 ' \
                             '! videoconvert ! video/x-raw,format=I420 ' \
                             '! x264enc speed-preset=ultrafast tune=zerolatency ' \
                             '! rtph264pay config-interval=1 name=pay0 pt=96' \
                             .format(opt.image_width, opt.image_height, self.fps)
        self.image_height=opt.image_height
        self.image_width= opt.image_width
        self.device_id = opt.device_id
        self.cap3 = Camera2D(camera_id="3", frame_width=self.image_width, frame_height=self.image_height,
                                 frame_rate=self.fps)
        self.cap2 = Camera2D(camera_id="2", frame_width=self.image_width, frame_height=self.image_height,
                                 frame_rate=self.fps) 
        self.cap1 = Camera2D(camera_id="1", frame_width=self.image_width, frame_height=self.image_height,
                                 frame_rate=self.fps)
        self.cap0 = Camera2D(camera_id="0", frame_width=self.image_width, frame_height=self.image_height,
                                 frame_rate=self.fps)       
                    
    # method to capture the video feed from the camera and push it to the
    # streaming buffer.

    def on_need_data(self, src, length):
        self.cap3.read()
        self.cap2.read() 
        self.cap1.read() 
        self.cap0.read()    
        # It is better to change the resolution of the camera 
        # instead of changing the image shape as it affects the image quality.
        frame_front = self.cap3.image_data.copy()  # front
        frame_left = self.cap2.image_data.copy()  # left
        frame_rear = self.cap1.image_data.copy()  # rear
        frame_right = self.cap0.image_data.copy()  # right
        width=int(opt.image_width/2)
        height=int(opt.image_height/2)
        frame_front = cv2.resize(frame_front, (width, height), \
            interpolation = cv2.INTER_LINEAR)
        frame_left = cv2.resize(frame_left, (width, height), \
            interpolation = cv2.INTER_LINEAR)
        frame_rear = cv2.resize(frame_rear, (width, height), \
            interpolation = cv2.INTER_LINEAR)
        frame_right = cv2.resize(frame_right, (width, height), \
            interpolation = cv2.INTER_LINEAR)
        framefb=frame_front
        position= rosSub.get_position()
        if(int(position[0])==0):  # not forward
            framefb=frame_rear
            frame_left,frame_right=frame_right,frame_left
            if(int(position[3])==0):
                frame_left=np.zeros((height,width,3), np.uint8)
            if(int(position[2])==0):
                frame_right=np.zeros((height,width, 3), np.uint8)
        elif(int(position[1])==0):
            framefb=frame_front
            if(int(position[2])==0):
                frame_left=np.zeros((height,width,3), np.uint8)
            if(int(position[3])==0):
                frame_right=np.zeros((height,width, 3), np.uint8)
        # framefb = np.vstack((frame_front,frame_rear))
        framefb = cv2.resize(framefb, (width, height), \
            interpolation = cv2.INTER_LINEAR)

        frame = np.hstack((frame_left,framefb,frame_right))
        frame = cv2.resize(frame, (2*width, 2*height), \
            interpolation = cv2.INTER_LINEAR)
        data = frame.tostring()
        buf = Gst.Buffer.new_allocate(None, len(data), None)
        buf.fill(0, data)
        buf.duration = self.duration
        timestamp = self.number_frames * self.duration
        buf.pts = buf.dts = int(timestamp)
        buf.offset = timestamp
        self.number_frames += 1
        retval = src.emit('push-buffer', buf)
        print('pushed buffer, frame {}, duration {} ns, durations {} s'.format(self.number_frames,
                                                                               self.duration,
                                                                               self.duration / Gst.SECOND))
        if retval != Gst.FlowReturn.OK:
            print(retval)
    # attach the launch string to the override method
    def do_create_element(self, url):
        return Gst.parse_launch(self.launch_string)
    
    # attaching the source element to the rtsp media
    def do_configure(self, rtsp_media):
        self.number_frames = 0
        appsrc = rtsp_media.get_element().get_child_by_name('source')
        appsrc.connect('need-data', self.on_need_data)

# Rtsp server implementation where we attach the factory sensor with the stream uri
class GstServer(GstRtspServer.RTSPServer):
    def __init__(self, **properties):
        super(GstServer, self).__init__(**properties)
        self.factory = SensorFactory()
        self.factory.set_shared(True)
        self.set_service(str(opt.port))
        self.get_mount_points().add_factory(opt.stream_uri, self.factory)
        self.attach(None)

# getting the required information from the user 
parser = argparse.ArgumentParser()
parser.add_argument("--device_id", required=True, help="device id for the \
                video device or video file location")
parser.add_argument("--fps", required=True, help="fps of the camera", type = int)
parser.add_argument("--image_width", required=True, help="video frame width", type = int)
parser.add_argument("--image_height", required=True, help="video frame height", type = int)
parser.add_argument("--port", default=8554, help="port to stream video", type = int)
parser.add_argument("--stream_uri", default = "/video_stream", help="rtsp video stream uri")
parser.add_argument("__name")
parser.add_argument("__log")
opt = parser.parse_args()

try:
    opt.device_id = int(opt.device_id)
except ValueError:
    pass

rosSub= RosSubscriber()
# initializing the threads and running the stream on loop.
GObject.threads_init()
Gst.init(None)
server = GstServer()
loop = GObject.MainLoop()
loop.run()
