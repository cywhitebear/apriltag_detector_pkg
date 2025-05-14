import os
import yaml
import math
from tf_transformations import quaternion_from_euler

def load_tag_poses(yaml_path):
    with open(yaml_path, 'r') as f:
        data = yaml.safe_load(f)

    tag_poses = {}
    for tag in data['tags']:
        tag_id = tag['id']
        pos = tag['position']
        rpy = tag['orientation_rpy']
        quat = quaternion_from_euler(rpy[0], rpy[1], rpy[2])

        tag_poses[tag_id] = {
            'position': tuple(pos),
            'orientation': quat
        }

    return tag_poses
