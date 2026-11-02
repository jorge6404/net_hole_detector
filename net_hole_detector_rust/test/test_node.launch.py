"""Prueba de integración del nodo: publica la imagen sintética de red y comprueba las salidas."""
import os
import time
import unittest

import cv2
import launch
import launch_ros.actions
import launch_testing.actions
import pytest
import rclpy
from sensor_msgs.msg import CameraInfo, CompressedImage, Image

from net_hole_detector_msgs_rust.msg import BoundingBoxArray, Detection3DArray, NetStats

HERE = os.path.dirname(os.path.abspath(__file__))
NS = '/nhd_test'


@pytest.mark.launch_test
def generate_test_description():
    node = launch_ros.actions.Node(
        package='net_hole_detector_rust',
        executable='net_hole_detector_node',
        namespace='nhd_test',
        output='screen',
        parameters=[
            os.path.join(HERE, '..', 'config', 'net_hole_detector.yaml'),
            {
                'inference.engine': 'cpu',
                'image_transport': 'raw',
                'yolo.period_s': 0.0,
                'yolo.confidence_threshold': 0.05,
            },
        ],
    )
    # Transporte compressed con la imagen remapeada a un topic más corto que el namespace
    compressed = launch_ros.actions.Node(
        package='net_hole_detector_rust',
        executable='net_hole_detector_node',
        namespace='nhd_test_compressed',
        output='screen',
        parameters=[
            os.path.join(HERE, '..', 'config', 'net_hole_detector.yaml'),
            {'inference.engine': 'cpu', 'image_transport': 'compressed',
             # números enteros en parámetros double: quien edita el YAML escribe 1, no 1.0
             'yolo.period_s': 1, 'max_z_age_s': 1, 'clahe.clip_limit': 3},
        ],
        remappings=[('image', '/nhd_cam'), ('image/compressed', '/nhd_cam/compressed')],
    )
    return launch.LaunchDescription([node, compressed, launch_testing.actions.ReadyToTest()])


def to_image_msg(img, encoding, frame_id):
    msg = Image()
    msg.header.frame_id = frame_id
    msg.height, msg.width = img.shape[:2]
    msg.encoding = encoding
    msg.step = img.strides[0]
    msg.data = img.tobytes()
    return msg


class TestNetHoleDetector(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.bgr = cv2.imread(os.path.join(HERE, 'data', 'net_sample.png'))

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('nhd_test_driver')
        self.pub_img = self.node.create_publisher(Image, NS + '/image', 1)
        self.pub_info = self.node.create_publisher(CameraInfo, NS + '/camera_info', 1)
        self.received = {'stats': [], 'boxes': [], 'dets': []}
        self.node.create_subscription(
            NetStats, NS + '/net_stats', lambda m: self.received['stats'].append(m), 10)
        self.node.create_subscription(
            BoundingBoxArray, NS + '/bounding_boxes', lambda m: self.received['boxes'].append(m), 10)
        self.node.create_subscription(
            Detection3DArray, NS + '/detections_3d', lambda m: self.received['dets'].append(m), 10)

    def tearDown(self):
        self.node.destroy_node()

    def run_until(self, done, image_msg, timeout=60.0):
        info = CameraInfo()
        info.width, info.height = image_msg.width, image_msg.height
        info.k = [1000.0, 0.0, 640.0, 0.0, 1000.0, 360.0, 0.0, 0.0, 1.0]
        info.header.frame_id = image_msg.header.frame_id
        end = time.time() + timeout
        next_pub = 0.0
        while time.time() < end and not done():
            if time.time() >= next_pub:
                stamp = self.node.get_clock().now().to_msg()
                image_msg.header.stamp = stamp
                info.header.stamp = stamp
                self.pub_info.publish(info)
                self.pub_img.publish(image_msg)
                next_pub = time.time() + 0.2
            rclpy.spin_once(self.node, timeout_sec=0.05)
        return done()

    def test_net_stats(self):
        msg = to_image_msg(self.bgr, 'bgr8', 'cam')
        self.assertTrue(self.run_until(lambda: self.received['stats'], msg), 'sin NetStats')
        s = self.received['stats'][0]
        self.assertGreater(s.scale_m_per_px, 0.0)
        self.assertAlmostEqual(
            s.median_hole_area_m2, s.median_hole_area_px * s.scale_m_per_px ** 2,
            delta=1e-3 * s.median_hole_area_m2)

    def test_detections(self):
        msg = to_image_msg(self.bgr, 'bgr8', 'cam')
        ok = self.run_until(lambda: self.received['boxes'] and self.received['dets'], msg)
        self.assertTrue(ok, 'sin BoundingBoxArray o Detection3DArray')
        boxes = self.received['boxes'][-1].boxes
        self.assertGreaterEqual(len(boxes), 1)
        for b in boxes:
            for v in (b.x, b.y, b.w, b.h, b.score):
                self.assertGreaterEqual(v, 0.0)
                self.assertLessEqual(v, 1.0)
        dets = self.received['dets'][-1]
        self.assertGreaterEqual(len(dets.detections), 1)
        self.assertGreater(dets.detections[0].z, 0.0)
        self.assertEqual(dets.header.frame_id, 'cam')

    def test_mono8(self):
        gray = cv2.cvtColor(self.bgr, cv2.COLOR_BGR2GRAY)
        msg = to_image_msg(gray, 'mono8', 'mono')
        ok = self.run_until(
            lambda: any(s.header.frame_id == 'mono' for s in self.received['stats']), msg)
        self.assertTrue(ok, 'una imagen mono8 no produjo NetStats')

    def test_compressed(self):
        pub = self.node.create_publisher(CompressedImage, '/nhd_cam/compressed', 1)
        got = []
        self.node.create_subscription(
            NetStats, '/nhd_test_compressed/net_stats', lambda m: got.append(m), 10)
        msg = CompressedImage()
        msg.header.frame_id = 'cam'
        msg.format = 'png'
        msg.data = cv2.imencode('.png', self.bgr)[1].tobytes()
        end = time.time() + 60.0
        while time.time() < end and not got:
            pub.publish(msg)
            rclpy.spin_once(self.node, timeout_sec=0.2)
        self.assertTrue(got, 'sin NetStats con transporte compressed')
