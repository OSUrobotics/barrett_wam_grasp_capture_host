#! /usr/bin/env python
import rospy
from std_srvs.srv import *
from std_msgs.msg import String
from std_msgs.msg import Empty as EmptyM
from geometry_msgs.msg import PoseStamped
from wam_srvs.srv import HandCommandBlkRequest, HandCommandBlkResponse, HandCommandBlk, JointRecordPlayback, CartPosMove, CartPosMoveRequest, CartPosMoveResponse, GravityComp, GravityCompRequest, GravityCompResponse, PoseMove, PoseMoveRequest, PoseMoveResponse, OrtnMove, OrtnMoveRequest, OrtnMoveResponse
from wam_msgs.msg import HandCommand, StampedString
from kinect_monitor import *

import sys
import os
import datetime
import time
import paramiko

from bag_manager import BagManager
from shared_globals import *


