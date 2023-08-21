# import sys, os
# from pathlib import Path
# import subprocess

# p = Path(__file__).resolve()
# panoptic_lib = p.parent.parent.parent.parent.parent / "4D-PLS"

# sys.path.append(str(panoptic_lib))
# os.chdir(panoptic_lib)

from test_stream import TestStream
import numpy as np
import os

from PyQt5 import QtWidgets, QtCore

from vispy.scene import SceneCanvas, visuals
from vispy.app import use_app

class StreamController(QtCore.QObject):

    new_data = QtCore.pyqtSignal(dict)
    finished = QtCore.pyqtSignal()
    
    def __init__(self):
        self.root_folder = os.getcwd()
        self.check_sequences()
        # self.lidar_subscriber = self.create_subscription(PointCloud2, '/ouster/points', self.subscriber_callback, 10)

    
    def check_sequences(self):
        self.streams_folder = os.path.join(self.root_folder, 'streams')
        # self.seq_folder = os.path.join(self.streams_folder, 'sequences')
        # print(os.getcwd())
        # if not os.path.exists(self.streams_folder):
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
        np.savetxt(pose_path, pose_data)
    
    # def subscriber_callback(self, pcl: PointCloud2):
    #     pcd_as_numpy_array = pc2.read_points(pcl, field_names=['x','y','z','intensity'], skip_nans=True)
    #     int_data = list(pcd_as_numpy_array)

    #     bin = np.empty(shape=[0,4])

    #     for x in int_data:
    #         bin = np.append(bin,[[x[0],x[1],x[2], x[3]]], axis = 0)

    #     bin = bin.reshape(-1,4).astype(np.float32)
    
    #     self.get_logger().info(str(bin))
    #     # bin.tofile(os.path.join(self.current_seq_folder, str(self.current_frame).zfill(6)+'.bin'))
    #     self.ts.update_dataset(bin)
    #     self.current_frame += 1
    

    def test_inference(self, path, ts: TestStream):

        bin = np.fromfile(path, dtype = np.float32)
        ts.update_dataset(bin)
        scan, label = ts.start_test()
        data_dict = {
            "scan":  scan,
            "label": label,
        }
        self.new_data.emit(data_dict)
        self.current_frame += 1
    
    def stop_data(self):
        print("Data source is quitting...")
        self._should_end = True

if __name__ == "__main__" :
    app = use_app("pyqt5")
    app.create()

    stream_node = StreamController()
    buffer_size = 5
    ts = TestStream(buffer_size=buffer_size)
    sequence_number = 5
    sequence_path = os.path.join(stream_node.root_folder, "streams", "sequences", str(sequence_number).zfill(2), "velodyne")
    pcd_list = np.sort(os.listdir(sequence_path))

    
    for pcd in pcd_list:
        stream_node.test_inference(os.path.join(sequence_path, pcd), ts)