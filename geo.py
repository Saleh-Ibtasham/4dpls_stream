import numpy as np
import os
import pandas as pd


np.random.seed(1)
mu = 0



def translate(pointcloud_path):
	#move all points by some random offset values
	#(x,y,z) to (x + dx,y + dy,z + dz)
	bin_pcd = np.fromfile(pointcloud_path, dtype=np.float32)
	points = bin_pcd.reshape((-1, 4))
	pc_df = pd.DataFrame(points)
	pc_df_copy = pc_df
	translate_val = [1, 5, 10]
	sigma_translate = np.random.choice(translate_val)
	normal_dist_translate = np.random.normal(mu,sigma_translate,1000)
	offsets = np.random.choice(normal_dist_translate, size = 3)
	pc_df_copy[0] += offsets[0]
	pc_df_copy[1] += offsets[1]
	pc_df_copy[2] += offsets[2]
	new_points = pc_df_copy.to_numpy()

	return new_points

def flip(pointcloud_path):
	# flip all point horizontally
	# (x,y,z) to (-x,y,z)
	bin_pcd = np.fromfile(pointcloud_path, dtype=np.float32)
	points = bin_pcd.reshape((-1, 4))
	pc_df = pd.DataFrame(points)
	pc_df_copy = pc_df
	pc_df_copy[0] = -1 * pc_df_copy[0]
	new_points = pc_df_copy.to_numpy()
	return new_points

def scale(pointcloud_path):
	# scale all data
	# (x,y,z) to (s*x,s*y, s*z)
	bin_pcd = np.fromfile(pointcloud_path, dtype=np.float32)
	points = bin_pcd.reshape((-1, 4))
	pc_df = pd.DataFrame(points)
	pc_df_copy = pc_df
	t_values = [0.05, 0.1, 0.25]
	t = np.random.choice(t_values)
	scaling_factor = np.random.uniform(low = 1-t, high = 1+t)

	pc_df_copy[0] = pc_df_copy[0] * scaling_factor
	pc_df_copy[1] = pc_df_copy[1] * scaling_factor
	pc_df_copy[2] = pc_df_copy[2] * scaling_factor
	new_points = pc_df_copy.to_numpy()
	return new_points

def rotate(pointcloud_path,):
	# rotate all point around z-axis
	# P'(x',y',z') = R * P(x,y,z)
	bin_pcd = np.fromfile(pointcloud_path, dtype=np.float32)
	points = bin_pcd.reshape((-1, 4))
	pc_df = pd.DataFrame(points)
	b_values = [-(np.pi / 8 ), -(np.pi / 4), (np.pi / 8 ), (np.pi / 4)]
	alpha = np.random.choice(b_values)
	rotation = [[np.cos(alpha) , -np.sin(alpha),0],
				[np.sin(alpha), np.cos(alpha), 0],
				[0 , 0 , 1]]
	new_point = []
	for p in points:
		rot_xyz=np.matmul(rotation,np.transpose(p[0:3]))
		new_point.append(np.transpose(rot_xyz))
	pc_new = pd.DataFrame(new_point)
	pc_new_copy = pc_new
	pc_new_copy[3] =pc_df[3]
	new_points = pc_new_copy.to_numpy()
	return new_points



def gnoise(pointcloud_path,sigma=5, clip=5):
	# add Gaussian Noise to all points
	bin_pcd = np.fromfile(pointcloud_path, dtype=np.float32)
	points = bin_pcd.reshape((-1, 4))
	pc_df = pd.DataFrame(points)

	assert (clip > 0)
	pcd_xyz = pc_df.iloc[:,0:3]
	Row, Col = pcd_xyz.shape
	jittered_point = np.clip(sigma * np.random.randn(Row, Col), -1 * clip, clip)

	jittered_point += pcd_xyz
	jittered_point[3] = pc_df[3]
	new_points = jittered_point.to_numpy()

	return new_points


def main(current_path,pc,aug_pc,geo,):
	"""

	:param current_path: sequence number
	:param pc: file contaning the .bin format data, here is 'velodyne'
	:param aug_pc: file that save the new point cloud data
	:param geo: geometric method used for augmentation
	:return:
	"""

	ori_path = os.path.join(current_path, pc)

	file_list = os.listdir(ori_path)

	aug_path = os.path.join(current_path, aug_pc)
	if os.path.exists(aug_path):
		pass
	else:
		os.makedirs(aug_path)
	for file in file_list:
		print(file)
		(filename, extension) = os.path.splitext(file)
		file_ = ori_path+'\\'+filename+'.bin'


		if geo =='translate':
			new_points = translate(file_)
		elif geo =='flip':
			new_points = flip(file_)
		elif geo =='scale':
			new_points = scale(file_)
		elif geo =='rotate':
			new_points = rotate(file_)
		elif geo == 'gnoise':
			new_points = gnoise(file_)
		else:
			print('Please choose a suitable geometric method')
			return
		aug_file_new = os.path.join(aug_path, file)
		new_points.astype("float32").tofile(aug_file_new)

# choose a method from: translate,flip, scale, rotate, gnoise
geo = 'rotate'

main('06','velodyne','velodyne_'+geo,geo,)


