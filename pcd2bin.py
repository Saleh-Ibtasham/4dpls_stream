import os
import numpy as np


def read_pcd(filepath):
    # read .pcd format data
    lidar = []
    with open(filepath,'r',) as f:
        line = f.readline().strip()

        while line:
            linestr = line.split(" ")

            if len(linestr) == 9 and linestr[0] != '#':
                linestr_convert = list(map(float, linestr))

                lidar.append(linestr_convert[0:4])
            line = f.readline().strip()
    return np.array(lidar)


def convert(pcdfolder, binfolder):
    # convert .pcd format data to .bin format point cloud data
    current_path = os.getcwd()
    ori_path = os.path.join(current_path, pcdfolder)
    file_list = os.listdir(ori_path)
    des_path = os.path.join(current_path, binfolder)
    if os.path.exists(des_path):
        pass
    else:
        os.makedirs(des_path)
    for file in file_list: 
        (filename,extension) = os.path.splitext(file)
        velodyne_file = os.path.join(ori_path, filename) + '.pcd'
        pl = read_pcd(velodyne_file)
        pl = pl.reshape(-1, 4).astype(np.float32)

        # remove '.'
        strls = filename.split(".")
        filename = strls[0] + strls[1]

        velodyne_file_new = os.path.join(des_path, filename) + '.bin'
        pl.tofile(velodyne_file_new)
        print(pl)
    
pcdfolder = '2022-05-04-11-20-11_converted'
binfolder = '2022-05-04-11-20-11_bin'

convert(pcdfolder, binfolder)


