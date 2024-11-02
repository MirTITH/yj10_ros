import array
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from cv_bridge import CvBridge
import numpy as np
from numpy.lib.recfunctions import structured_to_unstructured, unstructured_to_structured
from typing import Optional, Tuple, Iterable
from std_msgs.msg import Header
import sys

_DATATYPES = {}
_DATATYPES[PointField.INT8] = np.dtype(np.int8)
_DATATYPES[PointField.UINT8] = np.dtype(np.uint8)
_DATATYPES[PointField.INT16] = np.dtype(np.int16)
_DATATYPES[PointField.UINT16] = np.dtype(np.uint16)
_DATATYPES[PointField.INT32] = np.dtype(np.int32)
_DATATYPES[PointField.UINT32] = np.dtype(np.uint32)
_DATATYPES[PointField.FLOAT32] = np.dtype(np.float32)
_DATATYPES[PointField.FLOAT64] = np.dtype(np.float64)
DUMMY_FIELD_PREFIX = "unnamed_field"


def dtype_from_fields(fields: Iterable[PointField], point_step: Optional[int] = None) -> np.dtype:
    """
    Convert a Iterable of sensor_msgs.msg.PointField messages to a np.dtype.

    :param fields: The point cloud fields.
                   (Type: iterable of sensor_msgs.msg.PointField)
    :param point_step: Point step size in bytes. Calculated from the given fields by default.
                       (Type: optional of integer)
    :returns: NumPy datatype
    """
    # Create a lists containing the names, offsets and datatypes of all fields
    field_names = []
    field_offsets = []
    field_datatypes = []
    for i, field in enumerate(fields):
        # Datatype as numpy datatype
        datatype = _DATATYPES[field.datatype]
        # Name field
        if field.name == "":
            name = f"{DUMMY_FIELD_PREFIX}_{i}"
        else:
            name = field.name
        # Handle fields with count > 1 by creating subfields with a suffix consiting
        # of "_" followed by the subfield counter [0 -> (count - 1)]
        assert field.count > 0, "Can't process fields with count = 0."
        for a in range(field.count):
            # Add suffix if we have multiple subfields
            if field.count > 1:
                subfield_name = f"{name}_{a}"
            else:
                subfield_name = name
            assert subfield_name not in field_names, "Duplicate field names are not allowed!"
            field_names.append(subfield_name)
            # Create new offset that includes subfields
            field_offsets.append(field.offset + a * datatype.itemsize)
            field_datatypes.append(datatype.str)

    # Create dtype
    dtype_dict = {"names": field_names, "formats": field_datatypes, "offsets": field_offsets}
    if point_step is not None:
        dtype_dict["itemsize"] = point_step
    return np.dtype(dtype_dict)


def create_cloud(header: Header, fields: Iterable[PointField], points: Iterable, point_step: Optional[int] = None) -> PointCloud2:
    """
    Create a sensor_msgs.msg.PointCloud2 message.

    :param header: The point cloud header. (Type: std_msgs.msg.Header)
    :param fields: The point cloud fields.
                   (Type: iterable of sensor_msgs.msg.PointField)
    :param points: The point cloud points. List of iterables, i.e. one iterable
                   for each point, with the elements of each iterable being the
                   values of the fields for that point (in the same order as
                   the fields parameter)
    :param point_step: Point step size in bytes. Calculated from the given fields by default.
                       (Type: optional of integer)
    :return: The point cloud as sensor_msgs.msg.PointCloud2
    """
    # Check if input is numpy array
    if isinstance(points, np.ndarray):
        # Check if this is an unstructured array
        if points.dtype.names is None:
            assert all(
                fields[0].datatype == field.datatype for field in fields[1:]
            ), "All fields need to have the same datatype. Pass a structured NumPy array \
                    with multiple dtypes otherwise."
            # Convert unstructured to structured array
            points = unstructured_to_structured(points, dtype=dtype_from_fields(fields, point_step))
        else:
            assert points.dtype == dtype_from_fields(
                fields, point_step
            ), "PointFields and structured NumPy array dtype do not match for all fields! \
                    Check their field order, names and types."
    else:
        # Cast python objects to structured NumPy array (slow)
        points = np.array(
            # Points need to be tuples in the structured array
            list(map(tuple, points)),
            dtype=dtype_from_fields(fields, point_step),
        )

    # Handle organized clouds
    assert (
        len(points.shape) <= 2
    ), "Too many dimensions for organized cloud! \
            Points can only be organized in max. two dimensional space"
    height = 1
    width = points.shape[0]
    # Check if input points are an organized cloud (2D array of points)
    if len(points.shape) == 2:
        height = points.shape[1]

    # Convert numpy points to array.array
    memory_view = memoryview(points)
    casted = memory_view.cast("B")
    array_array = array.array("B")
    array_array.frombytes(casted)

    # Put everything together
    cloud = PointCloud2(
        header=header,
        height=height,
        width=width,
        is_dense=False,
        is_bigendian=sys.byteorder != "little",
        fields=fields,
        point_step=points.dtype.itemsize,
        row_step=(points.dtype.itemsize * width),
    )
    # Set cloud via property instead of the constructor because of the bug described in
    # https://github.com/ros2/common_interfaces/issues/176
    cloud.data = array_array
    return cloud


class PointCloudUtils:
    def __init__(self, intrinsics=None):
        super().__init__()
        self.bridge = CvBridge()
        self.intrinsics = intrinsics

    def set_intrinsics(self, intrinsics):
        self.intrinsics = intrinsics

    def convert_depth_msg_to_np(self, depth_msg: Image):
        return self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="16UC1")

    def generate_pcl_np(
        self,
        depth_image: np.ndarray,
        additional_mask: Optional[np.ndarray] = None,
    ) -> Tuple[np.ndarray, Optional[np.ndarray]]:
        if self.intrinsics is None:
            raise ValueError("Intrinsics not set")

        fx, fy, cx, cy = self.intrinsics[0], self.intrinsics[4], self.intrinsics[2], self.intrinsics[5]
        height, width = depth_image.shape
        points = []

        # Generate a grid of coordinates corresponding to the pixel locations
        u, v = np.meshgrid(np.arange(width), np.arange(height))

        # Set nan to zeros
        depth_image = np.nan_to_num(depth_image)

        # Mask out points where depth is zero
        mask = depth_image > 0

        z = depth_image[mask] / 1000.0  # Convert depth image from mm to meters
        u = u[mask]
        v = v[mask]

        if additional_mask is not None:
            pcl_mask = additional_mask[mask]
        else:
            pcl_mask = None

        # Calculate x, y, z coordinates
        x = (u - cx) * z / fx
        y = (v - cy) * z / fy

        # Stack x, y, z into a single array of shape (N, 3)
        points = np.vstack((x, y, z)).transpose()

        return points, pcl_mask

    def convert_pcl_to_msg(
        self,
        points: np.ndarray,
        header: Header,
    ) -> PointCloud2:

        point_cloud = PointCloud2()

        point_cloud.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]

        # if mask is not None:
        #     fields.append(PointField(name="mask", offset=12, datatype=PointField.FLOAT32, count=1))
        #     cloud = np.concatenate((points, mask[:, None]), axis=1)
        # else:
        #     cloud = points

        point_cloud.header = header

        if len(points.shape) == 3:
            point_cloud.height = points.shape[1]
            point_cloud.width = points.shape[0]
        else:
            point_cloud.height = 1
            point_cloud.width = len(points)

        point_cloud.is_bigendian = False
        point_cloud.point_step = 12
        point_cloud.row_step = point_cloud.point_step * points.shape[0]
        point_cloud.data = np.asarray(points, np.float32).tostring()

        return point_cloud
