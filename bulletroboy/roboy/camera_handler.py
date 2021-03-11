import pybullet as p
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

class CameraHandler():
    def __init__(self, roboy_id, caml_pub, camr_pub):
        self.roboy_id = roboy_id
        self.caml_pub = caml_pub
        self.camr_pub = camr_pub

        self.height = 480
        self.width = 640
        aspect = self.width/self.height

        fov, nearplane, farplane = 100, 0.01, 100
        self.projection_matrix = p.computeProjectionMatrixFOV(fov, aspect, nearplane, farplane)

        self.init_up_vector = (0, 0, 1)

        self.bridge = CvBridge()

        self.caml_link_id = 11
        self.camr_link_id = 12


    def publish_next_pics(self):
        left = self.camera(self.caml_link_id)
        right = self.camera(self.camr_link_id)
        left_pic = self.to_cv2(left[0],left[1],left[2],0)
        right_pic = self.to_cv2(right[0],right[1],right[2],1)

        vis = np.concatenate((left_pic, right_pic), axis=1)
        cv2.imshow('window', vis)
        cv2.waitKey(1)

        self.caml_pub.publish(self.bridge.cv2_to_compressed_imgmsg(left_pic))
        self.camr_pub.publish(self.bridge.cv2_to_compressed_imgmsg(right_pic))

    def camera(self,link_id):
        com_p, com_o, _, _, _, _ = p.getLinkState(self.roboy_id, link_id)
        com_t = list(com_p)
        com_p = list(com_p)

        com_t[1] = com_p[1] - 5

        rot_matrix = p.getMatrixFromQuaternion(com_o)
        rot_matrix = np.array(rot_matrix).reshape(3, 3)
        init_camera_vector = com_t
        camera_vector = rot_matrix.dot(init_camera_vector)
        up_vector = rot_matrix.dot(self.init_up_vector)

        view_matrix = p.computeViewMatrix(com_p, com_p + 0.1 * camera_vector, up_vector)
        w, h, img, depth, mask = p.getCameraImage(self.width, self.height, view_matrix, self.projection_matrix, shadow=0, renderer=p.ER_BULLET_HARDWARE_OPENGL,flags=p.ER_NO_SEGMENTATION_MASK)
        # p.addUserDebugLine(lineFromXYZ=com_p, lineToXYZ=camera_vector, lifeTime=0)
        return w,h,img

    def to_cv2(self,w,h,img,right):
        rgba_pic = np.array(img, np.uint8).reshape((h, w, 4))
        # if right:
            # rgba_pic = np.roll(rgba_pic, 300)
        pic = cv2.cvtColor(rgba_pic, cv2.COLOR_RGBA2BGR)
            # pic = pic[0:self.height,300:self.width]
        # else:
            # rgba_pic = np.roll(rgba_pic, -300)
            # pic = cv2.cvtColor(rgba_pic, cv2.COLOR_RGBA2BGR)
            # pic = pic[0:self.height,0:self.width-300]

        return pic