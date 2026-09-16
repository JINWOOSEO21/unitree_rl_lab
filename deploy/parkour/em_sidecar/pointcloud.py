"""Small, dependency-free-of-ROS decoder for sensor_msgs/PointCloud2 XYZ data."""
from __future__ import annotations

import numpy as np

_FLOAT32 = 7
_FLOAT64 = 8
_DTYPE_SIZE = {_FLOAT32: 4, _FLOAT64: 8}


def decode_xyz(msg) -> np.ndarray:
    """Decode finite XYZ points using PointCloud2 fields, strides, and byte order.

    Returns an ``(N, 3)`` float64 array. Points containing NaN or infinity are
    omitted because the elevation mapper cannot consume them safely.
    """
    width = int(msg.width)
    height = int(msg.height)
    point_step = int(msg.point_step)
    row_step = int(msg.row_step)
    if min(width, height, point_step, row_step) < 0:
        raise ValueError("PointCloud2 dimensions and strides must be non-negative")

    data = bytes(msg.data)
    if width == 0 or height == 0:
        if data:
            raise ValueError("empty PointCloud2 has non-empty data")
        return np.empty((0, 3), dtype=np.float64)
    if point_step == 0:
        raise ValueError("non-empty PointCloud2 has point_step=0")
    if row_step < width * point_step:
        raise ValueError("PointCloud2 row_step is smaller than width * point_step")
    expected_size = height * row_step
    if len(data) != expected_size:
        raise ValueError(
            f"PointCloud2 data length {len(data)} does not match height * row_step {expected_size}"
        )

    fields = {}
    for field in msg.fields:
        if field.name in fields:
            raise ValueError(f"PointCloud2 has duplicate {field.name} field")
        if field.name in ("x", "y", "z"):
            fields[field.name] = field
    missing = [name for name in ("x", "y", "z") if name not in fields]
    if missing:
        raise ValueError(f"PointCloud2 is missing fields: {', '.join(missing)}")

    endian = ">" if bool(msg.is_bigendian) else "<"
    coordinates = []
    for name in ("x", "y", "z"):
        field = fields[name]
        datatype = int(field.datatype)
        count = int(field.count)
        offset = int(field.offset)
        if datatype not in _DTYPE_SIZE:
            raise ValueError(f"PointCloud2 {name} must be FLOAT32 or FLOAT64")
        if count != 1:
            raise ValueError(f"PointCloud2 {name} field count must be 1")
        if offset < 0 or offset + _DTYPE_SIZE[datatype] > point_step:
            raise ValueError(f"PointCloud2 {name} field exceeds point_step")
        dtype = np.dtype(endian + ("f4" if datatype == _FLOAT32 else "f8"))
        view = np.ndarray(
            shape=(height, width), dtype=dtype, buffer=data, offset=offset,
            strides=(row_step, point_step),
        )
        coordinates.append(view.reshape(-1))

    points = np.column_stack(coordinates).astype(np.float64, copy=False)
    return points[np.isfinite(points).all(axis=1)]
