#
#
#      0=================================0
#      |    Kernel Point Convolutions    |
#      0=================================0
#
#
# ----------------------------------------------------------------------------------------------------------------------
#
#      Callable script to start a training on ModelNet40 dataset
#
# ----------------------------------------------------------------------------------------------------------------------
#
#      Hugues THOMAS - 06/03/2020
#


# ----------------------------------------------------------------------------------------------------------------------
#
#           Imports and global variables
#       \**********************************/
#

# Common libs
import signal
import os
import numpy as np
import sys
import torch

# Dataset
from datasets.SemanticKitti import *
from torch.utils.data import DataLoader

from utils.config import Config
from utils.tester import ModelTester
from models.architectures import KPCNN, KPFCNN
from visualize import Visualizer

from stitch_tracklets_stream import *

np.random.seed(0)
torch.manual_seed(0)
torch.cuda.manual_seed_all(0)


# ----------------------------------------------------------------------------------------------------------------------
#
#           Main Call
#       \***************/
#


# ----------------------------------------------------------------------------------------------------------------------
#
#           Main Call
#       \***************/
#

class TestStream():
    def __init__(self, buffer_size) -> None:
        self.chosen_log = 'results/Log_2022-08-30_11-04-33'
        chkp_idx = 1

        # Deal with 'last_XXXXXX' choices
        self.chosen_log = self.model_choice(self.chosen_log)

        ############################
        # Initialize the environment
        ############################

        # Set which gpu is going to be used
        GPU_ID = '0'
        if torch.cuda.device_count() > 1:
            GPU_ID = '0, 1'
            
        # changed by wangkang to set GPU
        # os.environ['CUDA_LAUNCH_BLOCKING'] = '1'
        os.environ['CUDA_VISIBLE_DEVICES'] = '0'
        
        if torch.cuda.is_available():
            self.device = torch.device("cuda:0")
        else:
            self.device = torch.device("cpu")

        
        torch.cuda.empty_cache()
        torch.cuda.set_device(int(GPU_ID))
        print(torch.cuda.current_device())
        
        print("Test_stream: torch is using: ", torch.cuda.get_device_name()) # uncomment when cuda

        ###############
        # Previous chkp
        ###############

        # Find all checkpoints in the chosen training folder
        self.chkp_path = os.path.join(self.chosen_log, 'checkpoints')
        chkps = [f for f in os.listdir(self.chkp_path) if 'chkp' in f]
        
        # Find which snapshot to restore
        if chkp_idx is None:
            chosen_chkp = 'current_chkp.tar'
        else:
            chosen_chkp = np.sort(chkps)[chkp_idx]
        chosen_chkp = os.path.join(self.chosen_log, 'checkpoints', chosen_chkp)
        print("Current chkp being used: ", chosen_chkp)
        # Initialize configuration class
        self.config = Config()
        self.config.load(self.chosen_log)


        ##################################
        # Change model parameters for test
        ##################################

        # Change parameters for the test here. For example, you can stop augmenting the input data.

        self.config.global_fet = False
        self.config.validation_size = 10
        self.config.input_threads = 16
        self.config.n_frames = 4
        self.config.n_test_frames = 1 #it should be smaller than config.n_frames
        if self.config.n_frames < self.config.n_test_frames:
            self.config.n_frames = self.config.n_test_frames
        self.config.big_gpu = False
        self.config.dataset_task = '4d_panoptic'
        #self.config.sampling = 'density'
        self.config.sampling = 'importance'
        self.config.decay_sampling = 'None'
        self.config.stride = 1
        self.config.first_subsampling_dl = 0.05
        self.config.buffer_size = buffer_size


        ##############
        # Prepare Data
        ##############

        print()
        print('Dataset Preparation')
        print('****************')
        self.set = "test"

        # Initiate dataset
        if self.config.dataset == 'SemanticKitti':
            self.test_dataset = SemanticKittiStreamDataset(self.config, set=self.set, balance_classes=False, seqential_batch=True)
        else:
            raise ValueError('Unsupported dataset : ' + self.config.dataset)

        print('\nModel Preparation')
        print('*****************')

        # Define network model
        self.stitch_stream = StreamStitchTracklet()
        t1 = time.time()
        if self.config.dataset_task == 'classification':
            self.net = KPCNN(self.config)
        elif self.config.dataset_task in ['cloud_segmentation', 'slam_segmentation']:
            self.net = KPFCNN(self.config, self.test_dataset.label_values, self.test_dataset.ignored_labels)
        else:
            raise ValueError('Unsupported dataset_task for testing: ' + self.config.dataset_task)

        # Define a visualizer class
        self.tester = ModelTester(self.net, chkp_path=chosen_chkp)
        print('Done in {:.1f}s\n'.format(time.time() - t1))

        print('\nStart test')
        print('**********\n')
        
        self.config.dataset_task = '4d_panoptic'
        
        return
    
    def start_test(self):

        return self.tester.panoptic_4d_test_stream(self.net, self.test_dataset, self.config, self.stitch_stream)
    
    def update_dataset(self, data):
        self.test_dataset.update_data(data)
    
    def model_choice(self, chosen_log):

        ###########################
        # Call the test initializer
        ###########################

        # Automatically retrieve the last trained model
        if chosen_log in ['last_ModelNet40', 'last_ShapeNetPart', 'last_S3DIS']:

            # Dataset name
            test_dataset = '_'.join(chosen_log.split('_')[1:])

            # List all training logs
            logs = np.sort([os.path.join('results', f) for f in os.listdir('results') if f.startswith('Log')])

            # Find the last log of asked dataset
            for log in logs[::-1]:
                log_config = Config()
                log_config.load(log)
                if log_config.dataset.startswith(test_dataset):
                    chosen_log = log
                    break

            if chosen_log in ['last_ModelNet40', 'last_ShapeNetPart', 'last_S3DIS']:
                raise ValueError('No log of the dataset "' + test_dataset + '" found')

        # Check if log exists
        if not os.path.exists(chosen_log):
            raise ValueError('The given log does not exists: ' + chosen_log)

        return chosen_log
    

"""
if __name__ == '__main__':

    ###############################
    # Choose the model to visualize
    ###############################

    #   Here you can choose which model you want to test with the variable test_model. Here are the possible values :
    #
    #       > 'last_XXX': Automatically retrieve the last trained model on dataset XXX
    #       > '(old_)results/Log_YYYY-MM-DD_HH-MM-SS': Directly provide the path of a trained model

    # chosen_log = 'results/Log_2022-08-18_12-24-09'#'results/Log_2022-07-24_16-39-44'  # => ModelNet40
    # chosen_log = 'results/Log_2022-08-26_15-56-19' 
    # # chosen_log = 'results/Log_2022-08-31_08-36-16'
    # chosen_log ='results/Log_2022-09-19_06-52-15' #origin
    # chosen_log = 'results/trhyper'
    chosen_log = 'results/Log_2022-08-30_11-04-33'
    # chosen_log = 'results/trsc600'
    # Choose the index of the checkpoint to load OR None if you want to load the current checkpoint
    chkp_idx = 1

    # Choose to test on validation or test split
    on_val = False

    # Deal with 'last_XXXXXX' choices
    chosen_log = model_choice(chosen_log)

    ############################
    # Initialize the environment
    ############################

    # Set which gpu is going to be used
    GPU_ID = '0'
    if torch.cuda.device_count() > 1:
        GPU_ID = '0, 1'
        
    # changed by wangkang to set GPU
    # os.environ['CUDA_LAUNCH_BLOCKING'] = '1'
    os.environ['CUDA_VISIBLE_DEVICES'] = '0'
    
    if torch.cuda.is_available():
        device = torch.device("cuda:0")
    else:
        device = torch.device("cpu")

    # print("torch is using: ", torch.cuda.get_device_name()) # uncomment when cuda
        
    # torch.cuda.set_device(int(GPU_ID))
    # print(torch.cuda.current_device())

    ###############
    # Previous chkp
    ###############

    # Find all checkpoints in the chosen training folder
    chkp_path = os.path.join(chosen_log, 'checkpoints')
    chkps = [f for f in os.listdir(chkp_path) if 'chkp' in f]
    
    # Find which snapshot to restore
    if chkp_idx is None:
        chosen_chkp = 'current_chkp.tar'
    else:
        chosen_chkp = np.sort(chkps)[chkp_idx]
    chosen_chkp = os.path.join(chosen_log, 'checkpoints', chosen_chkp)

    # Initialize configuration class
    config = Config()
    config.load(chosen_log)


    ##################################
    # Change model parameters for test
    ##################################

    # Change parameters for the test here. For example, you can stop augmenting the input data.

    config.global_fet = False
    config.validation_size = 10
    config.input_threads = 16
    config.n_frames = 4
    config.n_test_frames = 1 #it should be smaller than config.n_frames
    if config.n_frames < config.n_test_frames:
        config.n_frames = config.n_test_frames
    config.big_gpu = True
    config.dataset_task = '4d_panoptic'
    #config.sampling = 'density'
    config.sampling = 'importance'
    config.decay_sampling = 'None'
    config.stride = 1
    config.first_subsampling_dl = 0.05


    ##############
    # Prepare Data
    ##############

    print()
    print('Data Preparation')
    print('****************')
    set = "test"

    # Initiate dataset
    if config.dataset == 'SemanticKitti':
        test_dataset = SemanticKittiStreamDataset(config, set=set, balance_classes=False, seqential_batch=True)
        collate_fn = SemanticKittiCollate
    else:
        raise ValueError('Unsupported dataset : ' + config.dataset)

    # Data loader
    # test_loader = DataLoader(test_dataset,
    #                          batch_size=1,
    #                          collate_fn=collate_fn,
    #                          num_workers=0,#config.input_threads,
    #                          pin_memory=True)
    
    print("dataset length: ", len(test_dataset))

    # Calibrate samplers
    # test_sampler.calibration(test_loader, verbose=True)

    print('\nModel Preparation')
    print('*****************')

    # Define network model
    t1 = time.time()
    if config.dataset_task == 'classification':
        net = KPCNN(config)
    elif config.dataset_task in ['cloud_segmentation', 'slam_segmentation']:
        net = KPFCNN(config, test_dataset.label_values, test_dataset.ignored_labels)
    else:
        raise ValueError('Unsupported dataset_task for testing: ' + config.dataset_task)

    # Define a visualizer class
    tester = ModelTester(net, chkp_path=chosen_chkp)
    print('Done in {:.1f}s\n'.format(time.time() - t1))

    print('\nStart test')
    print('**********\n')
    
    config.dataset_task = '4d_panoptic'
    
    # Training
    if config.dataset_task == '4d_panoptic':
        tester.panoptic_4d_test_stream(net, test_dataset, config)
    else:
        raise ValueError('Unsupported dataset_task for testing: ' + config.dataset_task)
 """   