#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from threading import Thread
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
import os, sys
import torch
from PyQt5 import QtWidgets, QtCore
import time

from vispy.scene import SceneCanvas, visuals
from vispy.app import use_app

panoptic_lib = '/home/ros/Code/4dpls_stream/4D-PLS'
sys.path.append(panoptic_lib)

from test_stream import TestStream
from visualize import Visualizer

class StreamController(Node, QtCore.QObject):
    
    new_data = QtCore.pyqtSignal(dict)
    finished = QtCore.pyqtSignal()
    
    def __init__(self):
        # super().__init__("controller_node")
        Node.__init__(self, "controller_node")
        QtCore.QObject.__init__(self, None)
        self.get_logger().info("lidar controller started")
        self.root_folder = os.path.join(os.getcwd(), "")
        self.buffer_size = 5
        self.ts = TestStream(buffer_size=self.buffer_size)
        self.check_sequences(self.root_folder)
        self.lidar_subscriber = self.create_subscription(PointCloud2, '/ouster/points', self.subscriber_callback, 10)
    
    def check_sequences(self, root_folder):
        self.streams_folder = os.path.join(root_folder, 'streams')
        self.current_frame = 0
        calib_path = os.path.join(self.streams_folder, "calib.txt")
        with open(calib_path, 'w') as file:
            file.write('Tr: 1 0 0 0 0 1 0 0 0 0 1 0\n')
            file.close()
        pose_path = os.path.join(self.streams_folder, "poses.txt")
        pose_data = np.zeros([10,12]) # 10 rows for 10 point cloud poses
        pose_data[:,0] = 1
        pose_data[:,5] = 1
        pose_data[:,10] = 1
        np.savetxt(pose_path,pose_data)
    
    def subscriber_callback(self, pcl: PointCloud2):
        # checking the field names of the point cloud
        # filed_names = [field.name for field in pcl.fields]
        pcd_as_numpy_array = pc2.read_points_list(pcl, field_names=['x','y','z','intensity'], skip_nans=True)
        bin = np.array(pcd_as_numpy_array)

        bin = bin.reshape(-1,4).astype(np.float32)

        # self.get_logger().info(str(bin))
        
        self.ts.update_dataset(bin)
        time1 = time.perf_counter()
        scan, label = self.ts.start_test()
        time2 = time.perf_counter()
        print("Ros node inference time elapsed: ", time2 - time1)
        data_dict = {
            "scan": scan,
            "label": label,
        }
        self.new_data.emit(data_dict)
        time3 = time.perf_counter()
        print("Time for ui update elapsed: ", time3 - time2)
        # bin.tofile(os.path.join(self.current_seq_folder, str(self.current_frame).zfill(6)+'.bin'))
        self.current_frame += 1
    
    def stop_data(self):
        print("Data source is quitting...")
        self._should_end = True
    
        
def main(args=None):
    rclpy.init(args=args)

    app = use_app("pyqt5")
    app.create()
    win = Visualizer()
    node = StreamController()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    thread = Thread(target=executor.spin)

    node.new_data.connect(win.vis.update_scan)
    node.finished.connect(thread.join)
    win.closing.connect(node.stop_data)
    # thread.finished.connect(node.deleteLater)
    
    thread.start()
    
    try:
        win.show()
        sys.exit(app.run())
        # rclpy.spin(node)
        
    finally:
        print("Shutting down data input stream")
        torch.cuda.empty_cache()
        print("Waiting for data source to close gracefully...")
        node.destroy_node()
        executor.shutdown()

    # rclpy.shutdown()
        
        