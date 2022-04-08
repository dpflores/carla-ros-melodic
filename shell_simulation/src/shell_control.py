#!/usr/bin/env python
#
# Copyright (c) 2017 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
# Copyright (c) 2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
"""
Welcome to CARLA ROS manual control.

Use ARROWS or WASD keys for control.

    W            : throttle
    S            : brake
    AD           : steer
    Q            : toggle reverse
    Space        : hand-brake
    P            : toggle autopilot
    M            : toggle manual transmission
    ,/.          : gear up/down
    B            : toggle manual control

    F1           : toggle HUD
    H/?          : toggle help
    ESC          : quit
"""

from __future__ import print_function

import datetime
import math
from threading import Thread

import numpy
from transforms3d.euler import quat2euler

import ros_compatibility as roscomp
from ros_compatibility.node import CompatibleNode
from ros_compatibility.qos import QoSProfile, DurabilityPolicy

from carla_msgs.msg import CarlaStatus
from carla_msgs.msg import CarlaEgoVehicleInfo




from std_msgs.msg import Bool

#Control msg
from carla_msgs.msg import CarlaEgoVehicleControl

#Sensors msgs
from carla_msgs.msg import CarlaCollisionEvent
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Imu
from carla_msgs.msg import CarlaLaneInvasionEvent
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float32  #Speedometer
from carla_msgs.msg import CarlaEgoVehicleStatus
from sensor_msgs.msg import PointCloud2 #Lidar vlp16



# ==============================================================================
# -- World ---------------------------------------------------------------------
# ==============================================================================



class ShellVehicle(CompatibleNode):
    """
    defining the vehicle
    """

    def __init__(self):
        super(ShellVehicle, self).__init__("ShellVehicle")
        self._surface = None
        self.role_name = self.get_param("role_name", "ego_vehicle")
        self.controller = ShellControl(self.role_name, self)

        '''
        #Collision
        self.collision_subscriber = self.new_subscription(
            CarlaCollisionEvent, "/carla/{}/collision".format(self.role_name),
            self.on_collision, qos_profile=10)

        #Depth Image
        self.depth_image_subscriber = self.new_subscription(
            Image, "/carla/{}/depth_middle/image".format(self.role_name),
            self.on_depth_image, qos_profile=10)
        
        #Depth Info
        self.depth_info_subscriber = self.new_subscription(
            CameraInfo, "/carla/{}/depth_middle/camera_info".format(self.role_name),
            self.on_depth_info, qos_profile=10)
        
        #GNSS
        self.gnss_subscriber = self.new_subscription(
            NavSatFix, "/carla/{}/gnss".format(self.role_name),
            self.on_gnss, qos_profile=10)

        #IMU
        self.imu_subscriber = self.new_subscription(
            Imu, "/carla/{}/imu".format(self.role_name),
            self.on_imu, qos_profile=10)

        #Lane Invasion    
        self.lane_invasion_subscriber = self.new_subscription(
            CarlaLaneInvasionEvent, "/carla/{}/lane_invasion".format(self.role_name),
            self.on_lane_invasion, qos_profile=10)

        #Odometry   
        self.odometry_subscriber = self.new_subscription(
            Odometry, "/carla/{}/odometry".format(self.role_name),
            self.on_odometry, qos_profile=10)

        #Camera Image
        self.camera_image_subscriber = self.new_subscription(
            CompressedImage, "/carla/{}/rgb_front/image/compressed".format(self.role_name),
            self.on_camera_image, qos_profile=10)

        #Speedometer
        self.speedometer_subscriber = self.new_subscription(
            Float32, "/carla/{}/speedometer".format(self.role_name),
            self.on_speedometer, qos_profile=10)
        
        #Vehicle Status
        self.vehicle_status_subscriber = self.new_subscription(
            CarlaEgoVehicleStatus, "/carla/{}/vehicle_status".format(self.role_name),
            self.on_vehicle_status, qos_profile=10)

        #Vlp16_1 (Lidar)
        self.lidar_subscriber = self.new_subscription(
            PointCloud2, "/carla/{}/vlp16_1".format(self.role_name),
            self.on_lidar, qos_profile=10)
        '''


    def on_collision(self, data):
        """
        Callback on collision event
        """
        intensity = math.sqrt(data.normal_impulse.x**2 +
                              data.normal_impulse.y**2 + data.normal_impulse.z**2)

    def on_lane_invasion(self, data):
        """
        Callback on lane invasion event
        """
        text = []
        for marking in data.crossed_lane_markings:
            if marking is CarlaLaneInvasionEvent.LANE_MARKING_OTHER:
                text.append("Other")
            elif marking is CarlaLaneInvasionEvent.LANE_MARKING_BROKEN:
                text.append("Broken")
            elif marking is CarlaLaneInvasionEvent.LANE_MARKING_SOLID:
                text.append("Solid")
            else:
                text.append("Unknown ")
        

    def on_camera_image(self, image):
        """
        Callback when receiving a camera image
        """
        array = numpy.frombuffer(image.data, dtype=numpy.dtype("uint8"))
        array = numpy.reshape(array, (image.height, image.width, 4))
        array = array[:, :, :3]
        array = array[:, :, ::-1]
        

# ==============================================================================
# -- ShellControl -----------------------------------------------------------
# ==============================================================================


class ShellControl(object):
    """
    Handle Control
    """

    def __init__(self, role_name, node):
        self.role_name = role_name

        self.node = node

        self._autopilot_enabled = False
        self._control = CarlaEgoVehicleControl()
        self._steer_cache = 0.0
        

        
        fast_qos = QoSProfile(depth=10)

        self.auto_pilot_enable_publisher = self.node.new_publisher(
            Bool,
            "/carla/{}/enable_autopilot".format(self.role_name),
            qos_profile=fast_qos)


        self.vehicle_control_publisher = self.node.new_publisher(
            CarlaEgoVehicleControl,
            "/carla/{}/vehicle_control_cmd".format(self.role_name),
            qos_profile=fast_qos)

        # self.carla_status_subscriber = self.node.new_subscription(
        #     CarlaStatus,
        #     "/carla/status",
        #     self._on_new_carla_frame,
        #     qos_profile=10)


        self.set_autopilot(self._autopilot_enabled)

    def send_control(self, throttle, steer, brake, hand_brake, autopilot):
        """Send the control commands"""
        
        self._control.throttle = throttle
        self._control.steer = steer
        self._control.brake = brake
        self._control.hand_brake = hand_brake
        self._control.gear = 1 
        self._control.manual_gear_shift = False
        #print(self._control)
        self.set_autopilot(autopilot)
        if not autopilot:
            try:
               
                self.vehicle_control_publisher.publish(self._control) #Pub the control

            except Exception as error:
                self.node.logwarn("Could not send vehicle control: {}".format(error))

        
    
    def set_autopilot(self, enable):
        """
        enable/disable the autopilot
        """
        self.auto_pilot_enable_publisher.publish(Bool(data=enable))




# ==============================================================================
# -- main() --------------------------------------------------------------------
# ==============================================================================

def main(args=None):
    """
    main function
    """
    roscomp.init("control", args=args)

    # resolution should be similar to spawned camera with role-name 'view'
    resolution = {"width": 800, "height": 600}


    try:
        

        vehicle_node = ShellVehicle()

        executor = roscomp.executors.MultiThreadedExecutor()
        executor.add_node(vehicle_node)

        spin_thread = Thread(target=vehicle_node.spin)
        spin_thread.start()

        while roscomp.ok():
            
            vehicle_node.controller.send_control(throttle=0.4,steer=0,brake=0.,hand_brake=False, autopilot=False)

    except KeyboardInterrupt:
        roscomp.loginfo("User requested shut down.")
    finally:
        roscomp.shutdown()
        #spin_thread.join()


if __name__ == '__main__':
    main()
