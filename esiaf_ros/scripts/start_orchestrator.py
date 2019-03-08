#!/usr/bin/env python

from orchestrator.orchestrator import Orchestrator
from esiaf_ros.msg import *
import rospy

# config
import yaml
import sys

# initialize rosnode
rospy.init_node('orchestrator')


# read config
rospy.loginfo('Loading config...')
argv = sys.argv
if len(argv) < 2:
    rospy.logerr('Need path to configfile as first parameter!')
    exit('1')
path_to_config = argv[1]
data = yaml.safe_load(open(path_to_config))

# create the orchestrator
rospy.loginfo('Creating Orchestrator...')
orc = Orchestrator(
    remove_dead_rate=data['remove_dead_rate']
)

rospy.loginfo('Orchestrator ready!')
rospy.spin()

