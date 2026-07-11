"""Quaternion math helpers."""
import math

from geometry_msgs.msg import Quaternion


def _quat_normalize(q):
    x, y, z, w = [float(v) for v in q]
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm == 0.0:
        return (0.0, 0.0, 0.0, 1.0)
    return (x / norm, y / norm, z / norm, w / norm)


def _quat_multiply_raw(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def _quat_multiply(a, b):
    return _quat_normalize(_quat_multiply_raw(a, b))


def _quat_conjugate(q):
    x, y, z, w = q
    return (-x, -y, -z, w)


def _quat_rotate(q, point):
    px, py, pz = point
    rotated = _quat_multiply_raw(
        _quat_multiply_raw(_quat_normalize(q), (px, py, pz, 0.0)),
        _quat_conjugate(_quat_normalize(q)),
    )
    return (rotated[0], rotated[1], rotated[2])


def _quat_from_msg(q: Quaternion):
    return (q.x, q.y, q.z, q.w)


def _quat_to_msg(q):
    msg = Quaternion()
    msg.x, msg.y, msg.z, msg.w = _quat_normalize(q)
    return msg


def _quat_from_rotation_matrix(matrix):
    import numpy as np

    m = np.asarray(matrix, dtype=np.float64)
    trace = float(np.trace(m))
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        return _quat_normalize((
            (m[2, 1] - m[1, 2]) / s,
            (m[0, 2] - m[2, 0]) / s,
            (m[1, 0] - m[0, 1]) / s,
            0.25 * s,
        ))
    if m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
        return _quat_normalize((
            0.25 * s,
            (m[0, 1] + m[1, 0]) / s,
            (m[0, 2] + m[2, 0]) / s,
            (m[2, 1] - m[1, 2]) / s,
        ))
    if m[1, 1] > m[2, 2]:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
        return _quat_normalize((
            (m[0, 1] + m[1, 0]) / s,
            0.25 * s,
            (m[1, 2] + m[2, 1]) / s,
            (m[0, 2] - m[2, 0]) / s,
        ))
    s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
    return _quat_normalize((
        (m[0, 2] + m[2, 0]) / s,
        (m[1, 2] + m[2, 1]) / s,
        0.25 * s,
        (m[1, 0] - m[0, 1]) / s,
    ))


quat_normalize = _quat_normalize
quat_conjugate = _quat_conjugate
quat_rotate = _quat_rotate
