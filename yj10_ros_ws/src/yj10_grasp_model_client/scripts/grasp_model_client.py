#!/usr/bin/env python
import rospy
from sensor_msgs.msg import PointCloud2, Image, CameraInfo
from yj10_grasp_model_client.common_utils import FrameRateCounter, DataAligner, AlignedDataCollection, PointCloudUtils
from yj10_grasp_model_client.hdz_user_interface import HdzUserInterface
from yj10_grasp_model_client.moveit_group import MoveGroupPythonInterface
import numpy as np
from typing import Optional

import grpc
from grasp_model_grpc_msg import grasp_model_grpc_msg_pb2 as gmg_msg
from grasp_model_grpc_msg import grasp_model_grpc_msg_pb2_grpc
from grasp_model_grpc_msg.utils import PointCloud2Numpy, Numpy2PointCloud, GraspModeloseStamped2Ros
import tf2_ros
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped, Pose, PoseStamped
from scipy.spatial.transform import Rotation
import tf2_geometry_msgs


class GraspModelClient:
    def __init__(self, host: str = "localhost", port: int = 50051):
        self.channel = grpc.insecure_channel(f"{host}:{port}")
        self.grasp_stub = grasp_model_grpc_msg_pb2_grpc.GraspModelStub(self.channel)

    def generate_from_pointcloud(
        self,
        pcd: np.ndarray,
        frame_name: str = "",
        user_mask: Optional[np.ndarray] = None,
    ) -> gmg_msg.PoseStamped:
        pcd_msg = Numpy2PointCloud(pcd, frame_name)
        if user_mask is not None:
            pcd_msg.user_mask = user_mask.astype("bool").tobytes()
        response: gmg_msg.PoseStamped = self.grasp_stub.GenerateFromPointCloud(pcd_msg)
        return response

    def __del__(self):
        self.channel.close()


class GraspFrontEnd:
    def __init__(self, move_group_interface: MoveGroupPythonInterface):
        self.move_group_interface = move_group_interface

        rospy.Subscriber("/gz_cam/color/image_raw", Image, self.rgb_callback)
        rospy.Subscriber("/gz_cam/depth/image_raw", Image, self.depth_callback)
        rospy.Subscriber("/gz_cam/color/camera_info", CameraInfo, self.info_callback)

        self.pcl_util = PointCloudUtils()

        self.pcl_pub = rospy.Publisher("/grasp_front_end/point_cloud", PointCloud2, queue_size=1)

        self.data_aligner = DataAligner(
            data_configs=[
                {"name": "rgb", "maxlen": 5},
                {"name": "depth", "maxlen": 5},
            ],
            reference_data="rgb",
            callback=self.align_callback,
            timestamp_tolerance=1.0 / 30.0 / 2.0,
        )
        self.data_aligner_fps_counter = FrameRateCounter(1000)

        self.grasp_model_client = GraspModelClient("localhost", 50051)

        # self.timer = self.create_timer(1.0, self.timer_callback)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.tf_broadcaster = TransformBroadcaster()

        self.user_interface = HdzUserInterface(
            infer_callback=self.__infer_callback,
            move_to_home_callback=self.__move_to_home_callback,
            move_to_pose_callback=self.__move_to_callback,
            gripper_cmd_callback=self.__gripper_callback,
        )

    def info_callback(self, msg: CameraInfo):
        self.pcl_util.set_intrinsics(msg.K)

    def rgb_callback(self, msg: Image):
        self.data_aligner.add_data("rgb", msg)

    def depth_callback(self, msg: Image):
        self.data_aligner.add_data("depth", msg)

    def align_callback(self, data_collection: AlignedDataCollection):
        # self.data_aligner_fps_counter.tick()
        # self.data_aligner_fps_counter.print_info("Data aligner: ")

        rgb_msg: Image = data_collection.data_dict["rgb"].data
        depth_msg: Image = data_collection.data_dict["depth"].data

        rgb_np = CvBridge().imgmsg_to_cv2(rgb_msg, desired_encoding="bgr8")
        depth_np = CvBridge().imgmsg_to_cv2(depth_msg)
        depth_np = depth_np * 1000
        self.user_interface.update_rgb_depth(rgb_np, depth_np, rgb_msg.header.frame_id, rgb_msg.header.stamp)

    def broadcast_grasp_target(self, position, quat, frame_id, target_name="grasp_target"):
        t = TransformStamped()

        t.header.stamp = rospy.Time.now()
        t.header.frame_id = frame_id
        t.child_frame_id = target_name
        t.transform.translation.x = position.x
        t.transform.translation.y = position.y
        t.transform.translation.z = position.z
        t.transform.rotation.x = quat.x
        t.transform.rotation.y = quat.y
        t.transform.rotation.z = quat.z
        t.transform.rotation.w = quat.w

        self.tf_broadcaster.sendTransform(t)

    def __infer_callback(self, **kwargs):
        rospy.loginfo(f"Infer callback")
        mask: np.ndarray = kwargs["mask"]
        # rgb: np.ndarray = kwargs["rgb"]
        depth: np.ndarray = kwargs["depth"]
        frame_name: str = kwargs["frame_name"]

        if mask.any():
            try:
                pcl_np, pcl_mask = self.pcl_util.generate_pcl_np(depth, mask)
                response = self.grasp_model_client.generate_from_pointcloud(
                    pcd=pcl_np,
                    frame_name=frame_name,
                    user_mask=pcl_mask,
                )
                # Rotate the output from the grasp model to match the arm
                response.pose.orientation.CopyFrom(self.model_quat_2_arm_quat(response.pose.orientation))
                rospy.loginfo(str(response))
                self.broadcast_grasp_target(response.pose.position, response.pose.orientation, frame_name)
                return response
            except Exception as e:
                rospy.logerr(f"Error in timer_callback: {e}")

    def __move_to_callback(self, pose_stamped: gmg_msg.PoseStamped):
        rospy.loginfo(f"__move_to_callback")
        pose_ros = GraspModeloseStamped2Ros(pose_stamped)
        pose_ros = self.transform_pose(pose_ros, "world")
        self.move_group_interface.move_to_pose(pose_ros.pose)

    def __move_to_home_callback(self):
        rospy.loginfo(f"__move_to_home_callback")
        self.move_group_interface.move_to_named("down")

    def __gripper_callback(self, **kwargs):
        rospy.loginfo(f"__gripper_callback: {kwargs}")
        normalized_width = kwargs["normalized_width"]
        max_effort = kwargs["max_effort"]
        grasp_depth = kwargs["grasp_depth"]
        # self.arm_client.SetGripper(normalized_width, max_effort, grasp_depth)

    def model_quat_2_arm_quat(self, quat: gmg_msg.Quaternion) -> gmg_msg.Quaternion:
        """Rotate the output from the grasp model to match the arm

        Args:
            quat (gmg_msg.Quaternion): The quaternion from the grasp model

        Returns:
            gmg_msg.Quaternion: The quaternion for the arm
        """

        r = Rotation.from_quat([quat.x, quat.y, quat.z, quat.w])
        r = r * Rotation.from_euler("xyz", [np.pi / 2, 0, 0])
        new_quat = r.as_quat()
        return gmg_msg.Quaternion(x=new_quat[0], y=new_quat[1], z=new_quat[2], w=new_quat[3])

    def transform_pose(self, pose_stamped: PoseStamped, target_frame: str):
        """
        将 PoseStamped 转换到目标参考系

        :param pose_stamped: 输入的 PoseStamped 消息
        :param target_frame: 目标参考系，例如 "world"
        :return: 转换后的 PoseStamped 消息
        """

        try:
            # 获取从 pose_stamped 的原始参考系到目标参考系的转换
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                pose_stamped.header.frame_id,
                rospy.Time(0),
                timeout=rospy.Duration(1.0),
            )

            # 执行转换
            transformed_pose = tf2_geometry_msgs.do_transform_pose(pose_stamped, transform)
            return transformed_pose

        except Exception as e:
            rospy.logerr(f"Transform error: {e}")
            return None


def main():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our node so that multiple clients can
    # run simultaneously.
    rospy.init_node("grasp_model_client", anonymous=True)

    move_group_interface = MoveGroupPythonInterface("manipulator")
    grasp_front_end = GraspFrontEnd(move_group_interface)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == "__main__":
    main()
