import numpy as np
import os
import json

class FileManager:

    def __init__(self, dir_path: str):
        self.dir_path = dir_path
        self.files = sorted(os.listdir(dir_path))
    
    def get_link(self, file_idx):
        qry_file = self.files[file_idx]
        file_path = os.path.join(self.dir_path, qry_file)

        with open(file_path, 'r') as f:
            json_frame = json.load(f)
        pred_joints_img = np.array(json_frame['pred_joints_img'], dtype=np.float32) # p, b, q
        pred_body_pose = np.array(json_frame['pred_body_pose'], dtype=np.float32) # a
        pred_rotmat = np.array(json_frame['pred_rotmat'], dtype=np.float32) # R
        pred_betas = np.array(json_frame['pred_betas'], dtype=np.float32)
        pred_camera = np.array(json_frame['pred_camera'], dtype=np.float32)

