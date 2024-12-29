#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import CompressedImage, Image
import numpy as np
import cv2
import torch
from message_filters import ApproximateTimeSynchronizer, Subscriber

class Intrinsic:
    fx = 184.75
    fy = 184.75
    cx = 320.0
    cy = 180.0

class Extrinsic:
    # front rotation : 0deg, 0deg, 0deg (roll pitch yaw)
    # front translation : 1.65, 0, 1.5
    front_rotation = np.array([[1, 0, 0],
                               [0, 1, 0],
                               [0, 0, 1]], np.float64)
    front_translation = np.array([1.65, 0, 1.5], np.float64).reshape(3, 1)
    front2world = np.hstack((front_rotation, front_translation))
    front2world = np.vstack((front2world, [0, 0, 0, 1]))
    
    # back rotation : 0deg, 10deg, 180deg (yaw -> pitch)
    # back translation : 0.08, 0, 1.5
    rotation_pitch_minus_tendeg = np.array([[0.9848, 0, 0.1736],
                                      [0, 1, 0],
                                      [-0.1736, 0, 0.9848]], np.float64)
    back_rotation_yaw = np.array([[-1, 0, 0],
                                  [0, -1, 0],
                                  [0, 0, 1]], np.float64)
    back_rotation = np.matmul(back_rotation_yaw, rotation_pitch_minus_tendeg)
    back_translation = np.array([0.08, 0, 1.5], np.float64).reshape(3, 1)

    back2world = np.hstack((back_rotation, back_translation))
    back2world = np.vstack((back2world, [0, 0, 0, 1]))
    
    # left rotation : 0deg, 10deg, 90deg (yaw -> pitch)
    # left translation : 0.86, 0.37, 1.5
    left_rotation_yaw = np.array([[0, -1, 0],
                                  [1, 0, 0],
                                  [0, 0, 1]], np.float64)
    left_rotation = np.matmul(left_rotation_yaw, rotation_pitch_minus_tendeg)
    left_translation = np.array([0.86, 0.37, 1.5], np.float64).reshape(3, 1)
    left2world = np.hstack((left_rotation, left_translation))
    left2world = np.vstack((left2world, [0, 0, 0, 1]))
    
    # right rotation : 0deg, 10deg, 270deg (yaw -> pitch)
    # right translation : 0.86, -0.37, 1.5
    right_rotation_yaw = np.array([[0, 1, 0],
                                   [-1, 0, 0],
                                   [0, 0, 1]], np.float64)
    right_rotation = np.matmul(right_rotation_yaw, rotation_pitch_minus_tendeg)
    right_translation = np.array([0.86, -0.37, 1.5], np.float64).reshape(3, 1)
    right2world = np.hstack((right_rotation, right_translation))
    right2world = np.vstack((right2world, [0, 0, 0, 1]))
    
    R = np.array([[0, -1, 0], 
                [0, 0, -1], 
                [1, 0, 0]]) 
    inv_R = np.linalg.inv(R)
    


class cfg:
    x_range = (-10, 10)
    y_range = (-10, 10)
    bev_resolution = 0.1 # 10cm per pixel
    bev_size = (int((x_range[1] - x_range[0]) / bev_resolution), int((y_range[1] - y_range[0]) / bev_resolution))

class BEVGenerator:
    def __init__(self):
        rospy.init_node('BEV_generator', anonymous=True)
        self.front_cam_sub = Subscriber("/image_jpeg/front_cam_rgb", CompressedImage)
        self.back_cam_sub = Subscriber("/image_jpeg/back_cam_rgb", CompressedImage)
        self.left_cam_sub = Subscriber("/image_jpeg/left_cam_rgb", CompressedImage)
        self.right_cam_sub = Subscriber("/image_jpeg/right_cam_rgb", CompressedImage)
        self.front_depth_sub = Subscriber("/image/front_cam_depth", Image)
        self.back_depth_sub = Subscriber("/image/back_cam_depth", Image)
        self.left_depth_sub = Subscriber("/image/left_cam_depth", Image)
        self.right_depth_sub = Subscriber("/image/right_cam_depth", Image)
        
        self.front_image = None
        self.back_image = None
        self.left_image = None
        self.right_image = None
        self.front_depth = None
        self.back_depth = None
        self.left_depth = None
        self.right_depth = None
        self.is_syncronized = False
        
        self.bev_image = np.zeros((cfg.bev_size[0], cfg.bev_size[1], 3), dtype=np.uint8)
        
        
        '''
        TODO: Synchronize all 8 topics.
        You can fine-tune the slop parameter to adjust the synchronization time difference
        Default Value is not guaranteed to work properly
        '''
        self.sync = ApproximateTimeSynchronizer(
            [self.front_cam_sub, self.back_cam_sub, self.left_cam_sub, self.right_cam_sub,
             self.front_depth_sub, self.back_depth_sub, self.left_depth_sub, self.right_depth_sub],
            queue_size=100,
            slop=0.5  # Fine-tune this parameter
        )
        
        
        self.sync.registerCallback(self.synchronized_callback)
        self.rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            if self.is_syncronized:
                self.generate_bev()
                self.visualize_bev()
                self.is_syncronized = False

    def project(self, image, depth, extrinsic_matrix, inv_R):
        h, w, _ = image.shape
        depth = depth.reshape(h, w)

        image = torch.tensor(image, device='cuda', dtype=torch.float32)
        depth = torch.tensor(depth, device='cuda', dtype=torch.float32) / 255.0 * 15
        extrinsic_matrix = torch.tensor(extrinsic_matrix, device='cuda', dtype=torch.float32)
        inv_R = torch.tensor(inv_R, device='cuda', dtype=torch.float32)

        u = torch.arange(w, device='cuda').repeat(h, 1)
        v = torch.arange(h, device='cuda').unsqueeze(1).repeat(1, w)

        mask = depth > 0
        u, v, z = u[mask], v[mask], depth[mask]
    
        x = (u - Intrinsic.cx) * z / Intrinsic.fx
        y = (v - Intrinsic.cy) * z / Intrinsic.fy
        cam_points = torch.stack((x, y, z), dim=0)  # (3, N)

        world_points = inv_R @ cam_points  # (3, N)
        world_points = torch.cat((world_points, torch.ones((1, world_points.shape[1]), device='cuda')), dim=0)
        world_points = extrinsic_matrix @ world_points  # (4, N)
        
        x_bev = ((world_points[0] - cfg.x_range[0]) / cfg.bev_resolution).long()
        y_bev = ((world_points[1] - cfg.y_range[0]) / cfg.bev_resolution).long()

        mask = (0 <= x_bev) & (x_bev < cfg.bev_size[1]) & (0 <= y_bev) & (y_bev < cfg.bev_size[0])
        mask = mask & (world_points[2] < 2.0)
        x_bev, y_bev = x_bev[mask], y_bev[mask]

        colors = image[v[mask], u[mask]]
        self.bev_image[-x_bev, -y_bev] = colors

    def synchronized_callback(self, front_cam, back_cam, left_cam, right_cam,
                               front_depth, back_depth, left_depth, right_depth):
        # Decode and process synchronized messages
        self.front_image = self.decode_image(front_cam)
        self.back_image = self.decode_image(back_cam)
        self.left_image = self.decode_image(left_cam)
        self.right_image = self.decode_image(right_cam)
        self.front_depth = self.decode_depth(front_depth)
        self.back_depth = self.decode_depth(back_depth)
        self.left_depth = self.decode_depth(left_depth)
        self.right_depth = self.decode_depth(right_depth) 
        
        self.is_syncronized = True

    def generate_bev(self):
        self.bev_image = torch.zeros((cfg.bev_size[0], cfg.bev_size[1], 3), device='cuda', dtype=torch.float32)

        self.project(self.front_image, self.front_depth, Extrinsic.front2world, Extrinsic.inv_R)
        self.project(self.back_image, self.back_depth, Extrinsic.back2world, Extrinsic.inv_R)
        self.project(self.left_image, self.left_depth, Extrinsic.left2world, Extrinsic.inv_R)
        self.project(self.right_image, self.right_depth, Extrinsic.right2world, Extrinsic.inv_R)
        
        self.bev_image_cpu = torch.clamp(self.bev_image, 0, 255).cpu().numpy().astype(np.uint8)
        
    def visualize_bev(self):
        window_name = 'BEV'
        cv2.imshow(window_name, self.bev_image_cpu)
        cv2.waitKey(1)
        
    def decode_image(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        return cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

    def decode_depth(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        return np_arr
        
if __name__ == '__main__':
    try:
        BEVGenerator()
    except rospy.ROSInterruptException:
        pass