# Copyright 2023 Sameer Tuteja
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import sys

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from rcl_interfaces.msg import ParameterDescriptor

from sensor_msgs.msg import Image, CompressedImage
from sensor_msgs.msg import PointCloud2
import sensor_msgs.msg as sensor_msgs

from ifm3d_ros2.msg import RGBInfo, Extrinsics, Intrinsics

import open3d as o3d
import numpy as np

from cv_bridge import CvBridge

from scripts import transforms


class ColorPCLpub(Node):

    def __init__(self):
        super().__init__('colorPCLpub')

        #
        # Internal use
        #

        self.__colors_for_points = None
        self.__pcd_req = None
        self.__elapsed_wait_time = 0.5

        param_desc = ParameterDescriptor(
            description='Cloud stream speed. Default: 2Hz')

        self.declare_parameter('timer_period', 0.5, param_desc)
        timer_period = self.get_parameter_or(
            'timer_period', 0.5).get_parameter_value().double_value  # seconds

        self._timer = self.create_timer(
            timer_period, self._timer_publisher_pcl)

        qos_policy = QoSProfile(depth=1,
                                reliability=QoSReliabilityPolicy.RELIABLE,
                                durability=QoSDurabilityPolicy.VOLATILE)

        param_desc = ParameterDescriptor(
            description='Set this to true if uncompressed image')

        self.declare_parameter('uncompressed', False, param_desc)
        self.__is_uncompressed = self.get_parameter_or(
            'uncompressed', False).get_parameter_value().bool_value

        #
        # Subscribe
        #

        self.__rgb_c_image_sub = CompressedImage()
        self.__rgb_image_sub = Image()
        self.__distance_image_sub = Image()
        self.__rgb_info_msg_sub = RGBInfo()
        self.__extrinsic_3D_sub = Extrinsics()
        self.__intrinsics_3D_sub = Intrinsics()

        if self.__is_uncompressed is False:
            self._subscriber_rgb_c_image = self.create_subscription(
                CompressedImage, '/camera_2d/rgb_compressed',
                self._rgb_c_image_cb, qos_policy)

        if self.__is_uncompressed is True:
            self._subscriber_rgb_image = self.create_subscription(
                Image, '/camera_2d/rgb_uncompressed', self._rgb_image_cb,
                qos_policy)

        self._subscriber_distance_image = self.create_subscription(
            Image, '/camera_3d/distance', self._distance_image_cb,
            qos_policy)

        self._subscriber_RGB_INFO = self.create_subscription(
            RGBInfo, '/camera_2d/RGB_INFO', self._rgb_info_msg_cb,
            qos_policy)

        self._subscriber_extrinsic_3D = self.create_subscription(
            Extrinsics, '/camera_3d/extrinsics',
            self._extrinsic_3D_cb, qos_policy)

        self._subscriber_intrinsic_3D = self.create_subscription(
            Intrinsics, '/camera_3d/INTRINSIC_CALIB',
            self._intrinsic_3D_cb, qos_policy)

        #
        # Publish
        #

        self.__pcl_msg_pub = PointCloud2()

        self.__publisher_color_pcl = self.create_publisher(
            PointCloud2, '/colored_pcl', 10)

    def _rgb_c_image_cb(self, msg: CompressedImage()):
        self.__rgb_c_image_sub = msg

    def _rgb_image_cb(self, msg: Image()):
        self.__rgb_image_sub = msg

    def _distance_image_cb(self, msg: Image()):
        self.__distance_image_sub = msg

    def _rgb_info_msg_cb(self, msg: RGBInfo()):
        self.__rgb_info_msg_sub = msg

    def _extrinsic_3D_cb(self, msg: Extrinsics()):
        self.__extrinsic_3D_sub = msg

    def _intrinsic_3D_cb(self, msg: Intrinsics()):
        self.__intrinsics_3D_sub = msg

    def _get_color_points(self):

        if self.__is_uncompressed is True:
            if (len(self.__rgb_image_sub.data) == 0
                    or len(self.__distance_image_sub.data) == 0):
                return False

        elif self.__is_uncompressed is False:
            if (len(self.__rgb_c_image_sub.data) == 0
                    or len(self.__distance_image_sub.data) == 0):
                return False

        bridge = CvBridge()

        if self.__is_uncompressed is True:
            jpg = bridge.imgmsg_to_cv2(self.__rgb_image_sub, "rgb8")
        elif self.__is_uncompressed is False:
            jpg = bridge.compressed_imgmsg_to_cv2(
                self.__rgb_c_image_sub, 'rgb8')

        dis = bridge.imgmsg_to_cv2(
            self.__distance_image_sub, self.__distance_image_sub.encoding)

        modelID2D = self.__rgb_info_msg_sub.intrinsics.model_id
        invIntrinsic2D = \
            self.__rgb_info_msg_sub.inverse_intrinsics.model_parameters
        extrinsic2D = self.__rgb_info_msg_sub.extrinsics

        modelID3D = self.__intrinsics_3D_sub.model_id
        intrinsics3D = self.__intrinsics_3D_sub.model_parameters

        extrinsic3D = self.__extrinsic_3D_sub

        #
        # Following snippet is obtained from official documentation for o3r.
        #
        # Link for code:
        # https://github.com/ifm/documentation/blob/d844314f891a48b455ba16e1b3f333bee59bc2b3/SoftwareInterfaces/Toolbox/Registration2d3d/2D-3D_registration.py
        #

        # calculate 3D unit vectors corresponding to each pixel
        # of depth camera

        ux, uy, uz = transforms.intrinsic_projection(
            modelID3D, intrinsics3D, *dis.shape[::-1])

        # multiply unit vectors by depth of corresponding pixel

        x = (ux * dis).flatten()
        y = (uy * dis).flatten()
        z = (uz * dis).flatten()
        valid = dis.flatten() > 0.05

        for i, pt_valid in enumerate(valid):
            if not pt_valid:
                x[i] = y[i] = z[i] = 0.0

        # Restructure point cloud as sequence of points

        pcd_o3 = np.stack((x, y, z), axis=0)

        # Transform from optical coordinate system
        # to user coordinate system

        self.__pcd_req = transforms.translate(
            transforms.rotate_xyz(
                pcd_o3, extrinsic3D.rot_x,
                extrinsic3D.rot_y,
                extrinsic3D.rot_z),
            extrinsic3D.tx,
            extrinsic3D.ty,
            extrinsic3D.tz,
        )

        # convert to points in optics space
        # reverse internalTransRot

        r = np.array(
            [extrinsic2D.rot_x, extrinsic2D.rot_y, extrinsic2D.rot_z])
        t = np.array([extrinsic2D.tx, extrinsic2D.ty, extrinsic2D.tz])
        pcd_o2 = transforms.rot_mat(r).T.dot(
            self.__pcd_req - np.array(t)[..., np.newaxis])

        # Calculate 2D pixel coordinates for each 3D pixel

        pixels = np.round(transforms.inverse_intrinsic_projection(
            pcd_o2, invIntrinsic2D, modelID2D))

        # Get 2D jpg-color for each 3D-pixel
        # shape is Nx3 (for open3d)

        self.__colors_for_points = np.zeros((len(pixels[0]), 3))
        for i in range(len(self.__colors_for_points)):
            idxX = int(pixels[1][i])
            idxY = int(pixels[0][i])
            # Ignore invalid values
            if idxY > 1279 or idxX > 799 or idxY < 0 or idxX < 0:
                self.__colors_for_points[i, 0] = 126
                self.__colors_for_points[i, 1] = 126
                self.__colors_for_points[i, 2] = 126
            else:
                self.__colors_for_points[i, 0] = jpg.data[idxX, idxY, 0]
                self.__colors_for_points[i, 1] = jpg.data[idxX, idxY, 1]
                self.__colors_for_points[i, 2] = jpg.data[idxX, idxY, 2]

    def _timer_publisher_pcl(self):

        if self.__is_uncompressed is True:
            elapsed_time_rgb = self.get_clock().now(
            ) - Time.from_msg(self.__rgb_image_sub.header.stamp)
        elif self.__is_uncompressed is False:
            elapsed_time_rgb = self.get_clock().now(
            ) - Time.from_msg(self.__rgb_c_image_sub.header.stamp)

        elapsed_time_distance = self.get_clock().now(
        ) - Time.from_msg(self.__distance_image_sub.header.stamp)

        if ((elapsed_time_rgb > Duration(
            seconds=self.__elapsed_wait_time))
                or (elapsed_time_distance > Duration(
                    seconds=self.__elapsed_wait_time))):
            self.get_logger().info('No new data for image and distance topics')
            return

        if self._get_color_points() is False:
            self.get_logger().info('Waiting for image and distance data...')
            return

        o3d_pcd = o3d.geometry.PointCloud()
        o3d_pcd.points = o3d.utility.Vector3dVector(self.__pcd_req.T)
        o3d_pcd.colors = o3d.utility.Vector3dVector(
            self.__colors_for_points / 255)

        stacked_points = np.column_stack((o3d_pcd.points, o3d_pcd.colors))

        cloud_type = 'xyzrgb'
        ros_dtype = sensor_msgs.PointField.FLOAT32
        dtype = np.float32
        itemsize = np.dtype(dtype).itemsize

        data = stacked_points.astype(dtype).tobytes()

        fields = [sensor_msgs.PointField(
            name=n, offset=i*itemsize, datatype=ros_dtype, count=1)
            for i, n in enumerate(cloud_type)]

        self.__pcl_msg_pub.header.frame_id = \
            self.__distance_image_sub.header.frame_id
        self.__pcl_msg_pub.header.stamp = self.get_clock().now().to_msg()
        self.__pcl_msg_pub.height = self.__distance_image_sub.height
        self.__pcl_msg_pub.width = self.__distance_image_sub.width
        self.__pcl_msg_pub.is_bigendian = False
        self.__pcl_msg_pub.is_dense = True
        self.__pcl_msg_pub.fields = fields
        self.__pcl_msg_pub.point_step = (len(cloud_type) * itemsize)
        self.__pcl_msg_pub.row_step = (len(cloud_type) * itemsize
                                       * self.__distance_image_sub.height
                                       * self.__distance_image_sub.width)
        self.__pcl_msg_pub.data = data

        self.__publisher_color_pcl.publish(self.__pcl_msg_pub)


def main(args=None):

    rclpy.init(args=args)
    try:
        colorPCLpub = ColorPCLpub()
        rclpy.spin(colorPCLpub)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        sys.exit(1)
    finally:
        rclpy.try_shutdown()
        colorPCLpub.destroy_node()


if __name__ == '__main__':
    main()
