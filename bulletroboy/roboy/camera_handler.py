import pybullet as p
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

class CameraHandler():
    def __init__(self):
		height = 480
		width = 640
		aspect = width/height

		fov, nearplane, farplane = 100, 0.01, 100
		projection_matrix = p.computeProjectionMatrixFOV(fov, aspect, nearplane, farplane)

		init_up_vector = (0, 0, 1)

        self.bridge = CvBridge()

    def get_next_pics(self):
        left = camera(11)
        right = camera(12)
        left_pic = to_cv2(left[0],left[1],left[2],0)
        right_pic = to_cv2(right[0],right[1],right[2],1)

        vis = np.concatenate((left_pic, right_pic), axis=1)
        cv2.imshow('window', vis)
        cv2.waitKey(1)

        return bridge.cv2_to_compressed_imgmsg(left_pic), bridge.cv2_to_compressed_imgmsg(right_pic)

    def camera(self,link_id):
        com_p, com_o, _, _, _, _ = p.getLinkState(roboy, link_id)
        com_t = list(com_p)
        com_p = list(com_p)

        com_t[1] = com_p[1] - 5

        rot_matrix = p.getMatrixFromQuaternion(com_o)
        rot_matrix = np.array(rot_matrix).reshape(3, 3)
        init_camera_vector = com_t
        camera_vector = rot_matrix.dot(init_camera_vector)
        up_vector = rot_matrix.dot(init_up_vector)

        view_matrix = p.computeViewMatrix(com_p, com_p + 0.1 * camera_vector, up_vector)
        w, h, img, depth, mask = p.getCameraImage(width, height, view_matrix, projection_matrix, shadow=0, renderer=p.ER_BULLET_HARDWARE_OPENGL,flags=p.ER_NO_SEGMENTATION_MASK)
        # p.addUserDebugLine(lineFromXYZ=com_p, lineToXYZ=camera_vector, lifeTime=0)
        return w,h,img

    def to_cv2(self,w,h,img,right):
        rgba_pic = np.array(img, np.uint8).reshape((h, w, 4))
        # if right:
            # rgba_pic = np.roll(rgba_pic, 300)
        pic = cv2.cvtColor(rgba_pic, cv2.COLOR_RGBA2BGR)
            # pic = pic[0:height,300:width]
        # else:
            # rgba_pic = np.roll(rgba_pic, -300)
            # pic = cv2.cvtColor(rgba_pic, cv2.COLOR_RGBA2BGR)
            # pic = pic[0:height,0:width-300]

        return pic