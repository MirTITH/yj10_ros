import time
import rospy
import threading
import dataclasses
import queue
import cv2
from .common_utils import FrameRateCounter
from dataclasses import dataclass
import numpy as np
from typing import Optional

# from .hdz_grpc_arm_client import HdzGrpcArmClient
# from .hdz_grpc_msg import hdz_grpc_msg_pb2


@dataclass
class HdzUserInterfaceData:
    _lock: threading.Lock = threading.Lock()
    rgb: Optional[np.ndarray] = None
    depth: Optional[np.ndarray] = None
    timestamp: Optional[float] = None
    frame_name: Optional[str] = None

    def has(self, key):
        with self._lock:
            return getattr(self, key) is not None

    def get(self, key):
        with self._lock:
            return getattr(self, key)


class HdzUserInterface:
    def __init__(self, infer_callback):
        self.data = HdzUserInterfaceData()
        self.img_fps_counter = FrameRateCounter(1000)
        self.mask: Optional[np.ndarray] = None  # dtype=np.bool

        self.cv_window_name = "hdz_user_interface"
        self.rect_start = None
        self.rect_drawing = False

        self.grasp_pose = None
        self.infer_callback = infer_callback
        # self.arm_client = HdzGrpcArmClient("192.168.1.150", 9999)

        self.user_interface_thread = threading.Thread(target=self.__user_interface_entry)
        self.user_interface_thread.start()

    def __del__(self):
        self.user_interface_thread.join()

    def update_rgb_depth(self, rgb, depth, frame_name=None, timestamp=None):
        with self.data._lock:
            self.data.rgb = rgb
            self.data.depth = depth
            self.data.timestamp = timestamp
            self.data.frame_name = frame_name

    def __user_interface_entry(self):
        cv2.namedWindow(self.cv_window_name, cv2.WINDOW_GUI_NORMAL | cv2.WINDOW_AUTOSIZE)
        cv2.setMouseCallback(self.cv_window_name, self.__draw_rectangle)

        while not rospy.is_shutdown():
            # self.img_fps_counter.tick()
            # self.img_fps_counter.print_info("User Interface ")

            try:
                img: np.ndarray = self.data.get("rgb")

                if not self.__is_mask_valid(img.shape):
                    self.mask = self.__make_empty_mask(img.shape)

                img_with_mask = self.__get_img_with_mask(img, self.mask)

                if not self.mask.any():
                    self.__put_text(img_with_mask, "1. Use mouse to draw a rectangle", (10, 30))
                    self.__put_text(img_with_mask, "2. Right mouse button to infer", (10, 60))
                    self.__put_text(img_with_mask, "3. Middle mouse button to grasp", (10, 90))

                cv2.imshow(self.cv_window_name, img_with_mask)
                key = cv2.waitKey(10) & 0xFF

                if key == ord("0"):
                    rospy.loginfo("Key 0 pressed. Return to home position")
                    # self.arm_client.MoveToNamed("ready")
                elif key == ord("1"):
                    rospy.loginfo("Key 1 pressed. Open gripper")
                    # self.arm_client.SetGripper(1.0)
                elif key == ord("2"):
                    rospy.loginfo("Key 2 pressed. Close gripper")
                    # self.arm_client.SetGripper(0.0)

            except:
                time.sleep(0.1)

    def __put_text(self, img, text, position, font=cv2.FONT_HERSHEY_SIMPLEX, font_scale=1, color=(255, 255, 255)):
        cv2.putText(img, text, position, font, font_scale, color, 2, cv2.LINE_AA)

    def __make_empty_mask(self, rgb_shape):
        mask = np.zeros(rgb_shape[:2], dtype=bool)
        # mask[200:400, 300:500] = True
        return mask

    def __is_mask_valid(self, rgb_shape):
        return self.mask is not None and self.mask.shape == rgb_shape[:2]

    def __get_img_with_mask(self, img, mask, mask_color=(88, 61, 233), alpha=0.5):
        img_copy = img.copy()
        img_copy[mask] = img_copy[mask] * alpha + np.array(mask_color) * (1 - alpha)
        return img_copy

    # 回调函数，鼠标事件
    def __draw_rectangle(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.rect_drawing = True
            self.rect_start = (x, y)
            self.mask.fill(0)

        elif event == cv2.EVENT_MOUSEMOVE:
            if self.rect_drawing == True:
                self.mask.fill(0)
                start_x = min(self.rect_start[0], x)
                start_y = min(self.rect_start[1], y)
                end_x = max(self.rect_start[0], x)
                end_y = max(self.rect_start[1], y)
                self.mask[start_y:end_y, start_x:end_x] = True

        elif event == cv2.EVENT_LBUTTONUP:
            self.rect_drawing = False

        elif event == cv2.EVENT_MBUTTONDOWN:
            rospy.loginfo("Middle mouse button pressed")
            self.arm_client.MoveTo(self.grasp_pose)

        elif event == cv2.EVENT_RBUTTONDOWN:
            rospy.loginfo("Right mouse button pressed")
            with self.data._lock:
                rgb = self.data.rgb.copy()
                depth = self.data.depth.copy()
                timestamp = self.data.timestamp
                frame_name = self.data.frame_name

            mask = self.mask.copy()

            self.grasp_pose = self.infer_callback(
                mask=mask,
                rgb=rgb,
                depth=depth,
                timestamp=timestamp,
                frame_name=frame_name,
            )
