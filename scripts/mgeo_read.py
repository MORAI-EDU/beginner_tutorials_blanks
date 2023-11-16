#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import rospy

current_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(current_path)


from lib.mgeo.class_defs import *

load_path = os.path.normpath(os.path.join(current_path, 'lib/mgeo_data/kcity'))
mgeo_planner_map = MGeoPlannerMap.create_instance_from_json(load_path)

node_set = mgeo_planner_map.node_set
link_set = mgeo_planner_map.link_set
nodes=node_set.nodes
links=link_set.lines
print('---------------------------------------')
print('# of nodes: ', len(node_set.nodes))
print('# of links: ', len(link_set.lines))
print('---------------------------------------')