#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
import tf2_ros
import math
from tf_transformations import quaternion_from_euler, quaternion_from_matrix
import numpy as np
from geodesy import utm
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from sensor_msgs.msg import NavSatFix

class MocapToEnuBroadcaster(Node):
    def __init__(self):
        super().__init__('mocap_to_enu_tf_broadcaster')

        # Latched QoS profile
        qos_profile = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.SYSTEM_DEFAULT
        )

        self.publisher = self.create_publisher(NavSatFix, '/mocap/tank/gps', qos_profile)
        self.timer = self.create_timer(0.02, self.timer_cb)  # 50Hz

        # TF broadcaster
        self.br_mocap_tank = tf2_ros.StaticTransformBroadcaster(self)
        self.br_sim_tank = tf2_ros.StaticTransformBroadcaster(self)
        self.br_utm_tank = tf2_ros.StaticTransformBroadcaster(self)
        self.br_local_tank = tf2_ros.StaticTransformBroadcaster(self)
        self.br_mocap_mocap_enu = tf2_ros.StaticTransformBroadcaster(self)
        self.br_beckholmen = tf2_ros.StaticTransformBroadcaster(self)
        self.br_miqus = tf2_ros.StaticTransformBroadcaster(self)

        #### For real world
        # t_utm = TransformStamped()
        # t_utm.header.stamp = self.get_clock().now().to_msg()
        # t_utm.header.frame_id = 'utm_34_V'
        # t_utm.child_frame_id = 'tank_local'
        
        # utm_tank = utm.fromLatLong(59.3508558333, 18.0681783333)
        # t_utm.transform.translation.x = utm_tank.easting
        # t_utm.transform.translation.y = utm_tank.northing
        # t_utm.transform.translation.z = 0.0

        # q_utm = quaternion_from_euler(0, 0, 0)
        # t_utm.transform.rotation.x = q_utm[0]
        # t_utm.transform.rotation.y = q_utm[1]
        # t_utm.transform.rotation.z = q_utm[2]
        # t_utm.transform.rotation.w = q_utm[3]
        # self.br_utm_tank.sendTransform(t_utm)
        # self.get_logger().info('Broadcasted utm - tank_base transform')

        #### For sim
        #t_sim = TransformStamped()
        #t_sim.header.stamp = self.get_clock().now().to_msg()
        #t_sim.header.frame_id = 'KTHTank World/mocap'
        #t_sim.child_frame_id = 'mocap'
        #t_sim.transform.translation.x = 0.
        #t_sim.transform.translation.y = 0.
        #t_sim.transform.translation.z = 0.

        #q_utm = quaternion_from_euler(0, 0, 0)
        #t_sim.transform.rotation.x = q_utm[0]
        #t_sim.transform.rotation.y = q_utm[1]
        #t_sim.transform.rotation.z = q_utm[2]
        #t_sim.transform.rotation.w = q_utm[3]

        #self.br_sim_tank.sendTransform(t_sim)
        #self.get_logger().info('Broadcasted utm - tank_base transform')

        t_local = TransformStamped()
        t_local.header.stamp = self.get_clock().now().to_msg()
        t_local.header.frame_id = 'KTHTank World/tank_base'
        t_local.child_frame_id = 'tank_base'
        t_local.transform.translation.x = 0.0
        t_local.transform.translation.y = 0.0
        t_local.transform.translation.z = 0.0

        #q_local = quaternion_from_euler(0, 0, math.radians(212))
        q_local = quaternion_from_euler(0, 0, 0)
        t_local.transform.rotation.x = q_local[0]
        t_local.transform.rotation.y = q_local[1]
        t_local.transform.rotation.z = q_local[2]
        t_local.transform.rotation.w = q_local[3]

        self.br_local_tank.sendTransform(t_local)
        self.get_logger().info('Broadcasted utm - tank_base transform')

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'tank_base'
        t.child_frame_id = 'mocap'
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 2.8

        # 180 deg yaw + 90 deg roll = rotation from NED to ENU
        q = quaternion_from_euler(math.radians(180), 0, 0)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.br_mocap_tank.sendTransform(t)
        self.get_logger().info('Broadcasted mocap - tank_base transform')

        t_enu = TransformStamped()
        t_enu.header.stamp = self.get_clock().now().to_msg()
        t_enu.header.frame_id = 'mocap'
        t_enu.child_frame_id = 'map'
        t_enu.transform.translation.x = 0.0
        t_enu.transform.translation.y = 0.0
        t_enu.transform.translation.z = 0.0

        q_local = quaternion_from_euler(math.radians(180), 0, math.radians(90))
        t_enu.transform.rotation.x = q_local[0]
        t_enu.transform.rotation.y = q_local[1]
        t_enu.transform.rotation.z = q_local[2]
        t_enu.transform.rotation.w = q_local[3]

        self.br_mocap_mocap_enu.sendTransform(t_enu)
        self.get_logger().info('Broadcasted mocap -- mocap_enu transform')

        # Pose of the Miqus camera as exported by QTM in the calibration file:
        # <transform x="8436.609375" y="69.032700" z="-53.851807"
        #            r11="0.017026" r12="-0.997751" r13="-0.064837"
        #            r21="-0.285556" r22="0.057293" r23="-0.956648"
        #            r31="0.958211" r32="0.034802" r33="-0.283938"/>
        # QTM gives the translation in mm and the rotation row-major (r<row><col>)
        t_miqus = TransformStamped()
        t_miqus.header.stamp = self.get_clock().now().to_msg()
        t_miqus.header.frame_id = 'mocap'
        t_miqus.child_frame_id = 'miqus_camera'
        t_miqus.transform.translation.x = 8436.609375 / 1000.0
        t_miqus.transform.translation.y = 69.032700 / 1000.0
        t_miqus.transform.translation.z = -53.851807 / 1000.0

        # Building the rotation up one axis at a time. With the 180 deg roll of the
        # mocap frame taken into account the full rotation is RPY
        # [-6.988, 73.378, 86.588] deg. Roll and pitch added, yaw plus 90 deg.
        q_miqus = quaternion_from_euler(math.radians(-6.988),
                                        math.radians(-73.378),
                                        math.radians(86.588 + 90.0))

        # The full calibration matrix, for when we build back up to it:
        # mat_miqus = np.eye(4)
        # mat_miqus[:3, :3] = np.array([[0.017026, -0.997751, -0.064837],
        #                               [-0.285556, 0.057293, -0.956648],
        #                               [0.958211, 0.034802, -0.283938]])
        # q_miqus = quaternion_from_matrix(mat_miqus)
        t_miqus.transform.rotation.x = q_miqus[0]
        t_miqus.transform.rotation.y = q_miqus[1]
        t_miqus.transform.rotation.z = q_miqus[2]
        t_miqus.transform.rotation.w = q_miqus[3]

        self.br_miqus.sendTransform(t_miqus)
        self.get_logger().info('Broadcasted mocap -- miqus_camera transform')

        #t_enu = TransformStamped()
        #t_enu.header.stamp = self.get_clock().now().to_msg()
        #t_enu.header.frame_id = 'map'
        #t_enu.child_frame_id = 'beckholmen_map'
        #t_enu.transform.translation.x = 35.0
        #t_enu.transform.translation.y = 0.0
        #t_enu.transform.translation.z = -18.0

        #q_local = quaternion_from_euler(math.radians(10), 0, math.radians(-90))
        #t_enu.transform.rotation.x = q_local[0]
        #t_enu.transform.rotation.y = q_local[1]
        #t_enu.transform.rotation.z = q_local[2]
        #t_enu.transform.rotation.w = q_local[3]

        #self.br_beckholmen.sendTransform(t_enu)
        #self.get_logger().info('Broadcasted mocap -- mocap_enu transform')

        # Timer for broadcasting at 10 Hz
        # self.timer = self.create_timer(0.1, self.broadcast_transform)

    def timer_cb(self):

        # Create and publish one GPS message
        gps_msg = NavSatFix()
        gps_msg.header.stamp = self.get_clock().now().to_msg()
        gps_msg.header.frame_id = 'tank_base'
        gps_msg.latitude = 59.350925
        gps_msg.longitude = 18.068231
        gps_msg.altitude = 0.0

        self.publisher.publish(gps_msg)
        # self.get_logger().info('Latched GPS message published.')

        


def main(args=None):
    rclpy.init(args=args)
    node = MocapToEnuBroadcaster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
