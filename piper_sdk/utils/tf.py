from typing import Tuple
import math
from typing_extensions import (
    Literal,
)

# オイラー角順序エンコード表を定義
_AXES2TUPLE = {
    'sxyz': (0, 0, 0, 0),
    'rzyx': (0, 0, 0, 1),
}
axes = 'sxyz'
# 軸インデックス計算用
_NEXT_AXIS = [1, 2, 0, 1]
# 浮動小数点比較誤差しきい値を設定
_EPS = 1e-10

def normalize_quat(qx, qy, qz, qw):
    norm = math.sqrt(qx**2 + qy**2 + qz**2 + qw**2)
    return qx / norm, qy / norm, qz / norm, qw / norm

def quat_convert_euler(qx: float, qy: float, qz: float, qw: float) -> Tuple[float, float, float]:
                    #    axes: Literal['sxyz', 'rzyx'] = 'sxyz') -> Tuple[float, float, float]:
    """
    四元数 [x, y, z, w] をオイラー角 (roll, pitch, yaw) に変換。

    パラメータ:
        x, y, z, w - 四元数各成分
        axes - オイラー角の軸順序、'sxyz' または 'rzyx' をサポート

    戻り値:
        tuple(float, float, float): オイラー角 (roll, pitch, yaw)、単位はラジアン
    """
    
    # 軸順序設定
    try:
        firstaxis, parity, repetition, frame = _AXES2TUPLE[axes.lower()]
    except (KeyError, AttributeError):
        raise ValueError(f"Unsupported axes specification: '{axes}'. "
                        f"Only 'sxyz' and 'rzyx' are currently supported.")
    # 轴索引对应 x=0, y=1, z=2
    # 获取轴顺序索引
    i = firstaxis
    j = _NEXT_AXIS[i + parity]
    k = _NEXT_AXIS[i - parity + 1]
    qx, qy, qz, qw = normalize_quat(qx, qy, qz, qw)
    # 行列 M[i][j] 表現式：事前に3x3回転行列を展開
    M = [[0.0] * 3 for _ in range(3)]
    M[0][0] = 1 - 2*(qy**2 + qz**2)
    M[0][1] =     2*(qx*qy - qz*qw)
    M[0][2] =     2*(qx*qz + qy*qw)
    M[1][0] =     2*(qx*qy + qz*qw)
    M[1][1] = 1 - 2*(qx**2 + qz**2)
    M[1][2] =     2*(qy*qz - qx*qw)
    M[2][0] =     2*(qx*qz - qy*qw)
    M[2][1] =     2*(qy*qz + qx*qw)
    M[2][2] = 1 - 2*(qx**2 + qy**2)

    # オイラー角を計算
    if repetition:
        sy = math.sqrt(M[i][j] ** 2 + M[i][k] ** 2)
        if sy > _EPS:
            ax = math.atan2(M[i][j], M[i][k])
            ay = math.atan2(sy, M[i][i])
            az = math.atan2(M[j][i], -M[k][i])
        else:
            ax = math.atan2(-M[j][k], M[j][j])
            ay = math.atan2(sy, M[i][i])
            az = 0.0
    else:
        cy = math.sqrt(M[i][i] ** 2 + M[j][i] ** 2)
        if cy > _EPS:
            ax = math.atan2(M[k][j], M[k][k])
            ay = math.atan2(-M[k][i], cy)
            az = math.atan2(M[j][i], M[i][i])
        else:
            ax = math.atan2(-M[j][k], M[j][j])
            ay = math.atan2(-M[k][i], cy)
            az = 0.0

    # 角度方向を調整
    if parity:
        ax, ay, az = -ax, -ay, -az
    if frame:
        ax, az = az, ax

    return ax, ay, az

def euler_convert_quat(roll:float, pitch:float, yaw:float)->Tuple[float, float, float, float]:
    """
    オイラー角（roll, pitch, yaw）を四元数に変換。

    パラメータ:
        roll  - X軸周りの回転角（単位：ラジアン）
        pitch - Y軸周りの回転角（単位：ラジアン）
        yaw   - Z軸周りの回転角（単位：ラジアン）

    戻り値:
        list: 四元数 [x, y, z, w]
    """
    
    try:
        firstaxis, parity, repetition, frame = _AXES2TUPLE[axes.lower()]
    except (KeyError, AttributeError):
        raise ValueError(f"Unsupported axes specification: '{axes}'. "
                        f"Only 'sxyz' and 'rzyx' are currently supported.")
    # 轴索引对应 x=0, y=1, z=2
    # 获取轴顺序索引
    i = firstaxis
    j = _NEXT_AXIS[i + parity]
    k = _NEXT_AXIS[i - parity + 1]

    # 座標系調整
    if frame:
        roll, yaw = yaw, roll
    if parity:
        pitch = -pitch

    # 角度を半分に
    roll *= 0.5
    pitch *= 0.5
    yaw *= 0.5

    # 三角関数
    c_roll = math.cos(roll)
    s_roll = math.sin(roll)
    c_pitch = math.cos(pitch)
    s_pitch = math.sin(pitch)
    c_yaw = math.cos(yaw)
    s_yaw = math.sin(yaw)

    cc = c_roll * c_yaw
    cs = c_roll * s_yaw
    sc = s_roll * c_yaw
    ss = s_roll * s_yaw

    # 四元数 [x, y, z, w] を初期化
    q = [0.0, 0.0, 0.0, 0.0]

    if repetition:
        q[i] = c_pitch * (cs + sc)
        q[j] = s_pitch * (cc + ss)
        q[k] = s_pitch * (cs - sc)
        q[3] = c_pitch * (cc - ss)
    else:
        q[i] = c_pitch * sc - s_pitch * cs
        q[j] = c_pitch * ss + s_pitch * cc
        q[k] = c_pitch * cs - s_pitch * sc
        q[3] = c_pitch * cc + s_pitch * ss

    if parity:
        q[j] *= -1

    return q[0], q[1], q[2], q[3]  # [qx, qy, qz, qw]