#!/usr/bin/env python3
# This file is covered by the LICENSE file in the root of this project.

import argparse
import os
import yaml
from utils.laserscan import LaserScan, SemLaserScan
from utils.laserscanvis import LaserScanVis

from PyQt5 import QtWidgets, QtCore

class Visualizer(QtWidgets.QMainWindow):
  closing = QtCore.pyqtSignal()
  
  def __init__(self,  *args, **kwargs):
    super().__init__(*args, **kwargs)
    
    parser = argparse.ArgumentParser("")
    parser.add_argument(
        '--dataset', '-d',
        type=str,
        required=False,
        help='Dataset to visualize. No Default',
    )
    parser.add_argument(
        '--config', '-c',
        type=str,
        required=False,
        default="streams/semantic-kitti.yaml",
        help='Dataset config file. Defaults to %(default)s',
    )
    parser.add_argument(
        '--sequence', '-s',
        type=str,
        default="00",
        required=False,
        help='Sequence to visualize. Defaults to %(default)s',
    )
    parser.add_argument(
        '--predictions', '-p',
        type=str,
        default=None,
        required=False,
        help='Alternate location for labels, to use predictions folder. '
        'Must point to directory containing the predictions in the proper format '
        ' (see readme)'
        'Defaults to %(default)s',
    )
    parser.add_argument(
        '--ignore_semantics', '-i',
        dest='ignore_semantics',
        default=False,
        action='store_true',
        help='Ignore semantics. Visualizes uncolored pointclouds.'
        'Defaults to %(default)s',
    )
    parser.add_argument(
        '--do_instances', '-di',
        dest='do_instances',
        default=False,
        action='store_true',
        help='Visualize instances too. Defaults to %(default)s',
    )
    parser.add_argument(
        '--offset',
        type=int,
        default=0,
        required=False,
        help='Sequence to start. Defaults to %(default)s',
    )
    parser.add_argument(
        '--ignore_safety',
        dest='ignore_safety',
        default=False,
        action='store_true',
        help='Normally you want the number of labels and ptcls to be the same,'
        ', but if you are not done inferring this is not the case, so this disables'
        ' that safety.'
        'Defaults to %(default)s',
    )
    FLAGS, unparsed = parser.parse_known_args()

    # print summary of what we will do
    print("*" * 80)
    print("INTERFACE:")
    print("Dataset", FLAGS.dataset)
    print("Config", FLAGS.config)
    print("Sequence", FLAGS.sequence)
    print("Predictions", FLAGS.predictions)
    print("ignore_semantics", FLAGS.ignore_semantics)
    print("do_instances", FLAGS.do_instances)
    print("ignore_safety", FLAGS.ignore_safety)
    print("offset", FLAGS.offset)
    print("*" * 80)

    # open config file
    try:
      print("Opening config file %s" % FLAGS.config)
      CFG = yaml.safe_load(open(FLAGS.config, 'r'))
    except Exception as e:
      print(e)
      print("Error opening yaml file.")
      quit()

    # fix sequence name
    FLAGS.sequence = '{0:02d}'.format(int(FLAGS.sequence))

    # create a scan
    if FLAGS.ignore_semantics:
      scan = LaserScan(project=True)  # project all opened scans to spheric proj
    else:
      color_dict = CFG["color_map"]
      nclasses = len(color_dict)
      scan = SemLaserScan(nclasses, color_dict, project=True)

    # create a visualizer
    semantics = not FLAGS.ignore_semantics
    instances = FLAGS.do_instances
    # if not semantics:
    #   label_names = None
    self.vis = LaserScanVis(scan=scan,
                      offset=FLAGS.offset,
                      semantics=semantics, instances=instances and semantics)
    
    central_widget = QtWidgets.QWidget()
    main_layout = QtWidgets.QHBoxLayout()
    main_layout.addWidget(self.vis.canvas.native)
    
    central_widget.setLayout(main_layout)
    self.setCentralWidget(central_widget)
    
  def closeEvent(self, event):
    print("Closing main window!")
    self.closing.emit()
    return super().closeEvent(event)
