#!/bin/bash

scene_type=$1

template_file="ThreeRobots.ros2"
iter_per_scene=10

python run_eval.py  --template=$template_file --iter=$iter_per_scene 
# for (( i = 1; i < iter_per_scene; i++ )); do
    # python randomize_scenes.py --scene=$scene_type --template=$template_file
