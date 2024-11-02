from grasp_model_grpc_msg import grasp_model_grpc_msg_pb2
from grasp_model_grpc_msg.grasp_model_grpc_msg_pb2 import PointCloud
import numpy as np

def PointCloud2Numpy(msg: PointCloud) -> np.ndarray:
    if msg.height == 1:
        return np.frombuffer(msg.data, dtype=np.float32).reshape(msg.width, 3)
    else:
        return np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width, 3)

def Numpy2PointCloud(pcd: np.ndarray, frame_name: str = "") -> PointCloud:
    pcd_msg = PointCloud()
    pcd_msg.frame_name = frame_name
    if len(pcd.shape) == 2:
        pcd_msg.height = 1
        pcd_msg.width = pcd.shape[0]
    elif len(pcd.shape) == 3:
        pcd_msg.height = pcd.shape[0]
        pcd_msg.width = pcd.shape[1]
    else:
        raise ValueError("Input should only have 2 or 3 dimensions")

    pcd_msg.data = pcd.astype(np.float32).tobytes()
    return pcd_msg

def ConvertToGrpcPoint(input) -> grasp_model_grpc_msg_pb2.Point:
    if type(input) == np.ndarray:
        assert input.shape == (3,), "Input should only have 3 elements"
    else:
        assert len(input) == 3, "Input should only have 3 elements"
    
    return grasp_model_grpc_msg_pb2.Point(x=input[0], y=input[1], z=input[2])

def ConvertToGrpcQuaternion(input) -> grasp_model_grpc_msg_pb2.Quaternion:
    if type(input) == np.ndarray:
        assert input.shape == (4,), "Input should only have 4 elements"
    else:
        assert len(input) == 4, "Input should only have 4 elements"
    
    return grasp_model_grpc_msg_pb2.Quaternion(x=input[0], y=input[1], z=input[2], w=input[3])
