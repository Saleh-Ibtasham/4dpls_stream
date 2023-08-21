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

from vispy.scene import SceneCanvas, visuals
from vispy.app import use_app

panoptic_lib = '/home/ros/Desktop/Code/4dpls_stream/4D-PLS'
sys.path.append(panoptic_lib)

from test_stream import TestStream
from visualize import Visualizer

# class DataSource(QtCore.QObject):
    
#     new_data = QtCore.pyqtSignal(dict)
#     finished = QtCore.pyqtSignal()
    
#     def __init__(self, parent = None):
#         super().__init__(parent)
#         pass
#     def run_data_creation(self, data_dict):
#         self.new_data.emit(data_dict)
#         QtCore.QTimer.singleShot(0, self.run_data_creation)
        
#     def stop_data(self):
#         print("Data source is quitting...")
#         self._should_end = True

class StreamController(Node, QtCore.QObject):
    
    new_data = QtCore.pyqtSignal(dict)
    finished = QtCore.pyqtSignal()
    
    def __init__(self):
        # super().__init__("controller_node")
        Node.__init__(self, "controller_node")
        QtCore.QObject.__init__(self, None)
        self.get_logger().info("lidar controller started")
        self.root_folder = os.path.join(os.getcwd(), "4D-PLS/")
        self.buffer_size = 5
        self.ts = TestStream(buffer_size=self.buffer_size)
        self.check_sequences(self.root_folder)
        self.lidar_subscriber = self.create_subscription(PointCloud2, '/ouster/points', self.subscriber_callback, 10)
    
    def check_sequences(self, root_folder):
        self.streams_folder = os.path.join(root_folder, 'streams')
        # self.seq_folder = os.path.join(self.streams_folder, 'sequences')
        # print(os.getcwd())
        # if not os.path.exists(self.seq_folder):
        #     os.makedirs(self.seq_folder, exist_ok= True)
        # list_of_sequences = np.sort(os.listdir(self.seq_folder))
        
        # if len(list_of_sequences) <= 0:
        #     current_sequence = 0
        # else:
        #     current_sequence = int(list_of_sequences[-1]) + 1
            
        # self.current_seq_folder = os.path.join(self.seq_folder, str(current_sequence).zfill(2), "velodyne")
        # if not os.path.exists(self.current_seq_folder):
        #     os.makedirs(self.current_seq_folder, exist_ok=False)
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
        pcd_as_numpy_array = pc2.read_points(pcl, field_names=['x','y','z','intensity'], skip_nans=True)
        int_data = list(pcd_as_numpy_array)
        
        # xyz = np.empty(shape=[0,3])
        # rgb = np.empty(shape=[0,3])
        bin = np.empty(shape=[0,4])

        for x in int_data:
            # test = x[3] 
            # cast float32 to int so that bitwise operations are possible
            # s = struct.pack('>f' ,test)
            # i = struct.unpack('>l',s)[0]
            # you can get back the float value by the inverse operations
            # pack = ctypes.c_uint32(i).value
            # r = (pack & 0x00FF0000)>> 16
            # g = (pack & 0x0000FF00)>> 8
            # b = (pack & 0x000000FF)
            # prints r,g,b values in the 0-255 range
                        # x,y,z can be retrieved from the x[0],x[1],x[2]
            # xyz = np.append(xyz,[[x[0],x[1],x[2]]], axis = 0)
            # rgb = np.append(rgb,[[r,g,b]], axis = 0)
            bin = np.append(bin,[[x[0],x[1],x[2], x[3]]], axis = 0)
        # out_pcd = o3d.geometry.PointCloud()    
        # out_pcd.points = o3d.utility.Vector3dVector(xyz)
        # out_pcd.colors = o3d.utility.Vector3dVector(rgb)
        bin = bin.reshape(-1,4).astype(np.float32)
        
        # bin_pcd = out_pcd.reshape(-1, 4).astype(np.float32)
        
        
        # self.get_logger().info(str(filed_names))
        # self.get_logger().info(str(int_data[0]))
        # self.get_logger().info(str(np.asarray(out_pcd.points)))
        # self.get_logger().info(str(np.asarray(out_pcd.colors)))
        self.get_logger().info(str(bin))
        self.ts.update_dataset(bin)
        scan, label = self.ts.start_test()
        data_dict = {
            "scan": scan,
            "label": label,
        }
        # self.data_source.run_data_creation(data_dict=data_dict)
        self.new_data.emit(data_dict)
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
    #window_thread = QtCore.QThread()
    #win.moveToThread(self.window_thread)
    node.new_data.connect(win.vis.update_scan)
    # data_thread.started.connect(self.subscriber_callback)
    #window_thread.started.connect(self.win.show)
    node.finished.connect(thread.join)
    win.closing.connect(node.stop_data)
    # thread.finished.connect(node.deleteLater)
    #window_thread.start()
    
    thread.start()
    
    try:
        win.show()
        sys.exit(app.run())
        # rclpy.spin(node)
        
    finally:
        print("Shutting down data input stream")
        torch.cuda.empty_cache()
        print("Waiting for data source to close gracefully...")
        # node.data_thread.quit()
        # node.data_thread.wait(5000)
        node.destroy_node()
        executor.shutdown()

    # rclpy.shutdown()
        
        