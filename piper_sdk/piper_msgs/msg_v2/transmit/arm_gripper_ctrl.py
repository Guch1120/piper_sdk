#!/usr/bin/env python3
# -*-coding:utf8-*-
from typing_extensions import (
    Literal,
)
class ArmMsgGripperCtrl:
    '''
    msg_v2_transmit
    
    グリッパー制御コマンド
    
    CAN ID:
        0x159
    
    Args:
        grippers_angle: グリッパーストローク
        grippers_effort: グリッパートルク、範囲0-5000、0-5N/mに対応
        status_code: 
                0x00無効;
                0x01有効;
                0x02無効化してエラークリア;
                0x03有効化してエラークリア.
        set_zero: グリッパーゼロ点設定
                0x00無効値
                0xAEゼロ点設定
    
    ビット記述:
    
        Byte 0 grippers_angle: int32, 単位 0.001mm, グリッパーストローク、整数で表示。
        Byte 1
        Byte 2
        Byte 3
        Byte 4 grippers_effort: uint16, 単位 0.001N/m, グリッパートルク、整数で表示。
        Byte 5
        Byte 6 status_code: uint8, グリッパーステータスコード, 有効/無効/エラークリア;
                0x00無効;
                0x01有効;
                0x02無効化してエラークリア;
                0x03有効化してエラークリア.
        Byte 7 set_zero: uint8, 現在位置を0点に設定
                0x00無効値
                0xAEゼロ点設定
    '''
    '''
    msg_v2_transmit
    
    Gripper Control Command

    CAN ID:
        0x159

    Args:
        grippers_angle: Gripper stroke, represented as an integer, unit: 0.001mm.
        grippers_effort: Gripper torque, represented as an integer, unit: 0.001N·m.Range 0-5000,corresponse 0-5N/m
        status_code: 
            0x00: Disable;
            0x01: Enable;
            0x03: Enable with clear error;
            0x02: Disable with clear error.
        set_zero: Set the current position as the zero point.
            0x00: Invalid;
            0xAE: Set zero.

    Bit Description:

        Byte 0-3 grippers_angle: int32, unit: 0.001°, represents the gripper stroke.
        Byte 4-5 grippers_effort: uint16, unit: 0.001N·m, represents the gripper torque.
        Byte 6 status_code: uint8, gripper status code for enable/disable/clear error.
            0x00: Disable;
            0x01: Enable;
            0x03: Enable with clear error;
            0x02: Disable with clear error.
        Byte 7 set_zero: uint8, flag to set the current position as the zero point.
            0x00: Invalid;
            0xAE: Set zero.
    '''
    def __init__(self, 
                 grippers_angle: int = 0, 
                 grippers_effort: int = 0, 
                 status_code: Literal[0x00, 0x01, 0x02, 0x03] = 0,
                 set_zero: Literal[0x00, 0xAE] = 0):
        if status_code not in [0x00, 0x01, 0x02, 0x03]:
            raise ValueError(f"'status_code' Value {status_code} out of range [0x00, 0x01, 0x02, 0x03]")
        if not (0 <= grippers_effort <= 5000):
            raise ValueError(f"'grippers_effort' Value {grippers_effort} out of range 0-5000")
        if set_zero not in [0x00, 0xAE]:
            raise ValueError(f"'set_zero' Value {set_zero} out of range [0x00,0xAE]")
        self.grippers_angle = grippers_angle
        self.grippers_effort = grippers_effort
        self.status_code = status_code
        self.set_zero = set_zero

    def __str__(self):
        return (f"ArmMsgGripperCtrl(\n"
                f"  grippers_angle: {self.grippers_angle}, {self.grippers_angle * 0.001:.3f},\n"
                f"  grippers_effort: {self.grippers_effort} \t {self.grippers_effort * 0.001:.3f},\n"
                f"  status_code: {self.status_code},\n"
                f"  set_zero: {self.set_zero}\n"
                f")")

    def __repr__(self):
        return self.__str__()
