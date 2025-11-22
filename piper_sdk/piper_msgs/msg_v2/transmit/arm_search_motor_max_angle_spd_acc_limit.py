#!/usr/bin/env python3
# -*-coding:utf8-*-
from typing_extensions import (
    Literal,
)

class ArmMsgSearchMotorMaxAngleSpdAccLimit:
    '''
    msg_v2_transmit
    
    モーター角度/最大速度/最大加速度制限クエリコマンド

    CAN ID:
        0x472

    Args:
        motor_num: 関節モーター番号,1-6
        search_content: クエリ内容,0x01-モーター角度/最大速度クエリ,0x02-モーター最大加速度制限クエリ

    ビット記述:
    
        :Byte 0 motor_num: uint8, 関節モーター番号。
                値域 1-6,1-6 は関節ドライバ番号
        :Byte 1 search_content: uint8, クエリ内容。
                0x01 : モーター角度/最大速度クエリ
                0x02 : モーター最大加速度制限クエリ
    '''
    '''
    msg_v2_transmit
    
    Motor Angle/Max Speed/Max Acceleration Limit Query Command

    CAN ID:
        0x472

    Args:
        motor_num: Motor joint number.
        search_content: Query content.

    Bit Description:

        Byte 0: uint8, motor joint number.
            Value range: 1-6.
                1-6: Represent joint driver numbers.
        Byte 1: uint8, query content.
            0x01: Query motor angle/max speed.
            0x02: Query motor max acceleration limit.
    '''
    def __init__(self, 
                 motor_num: Literal[1, 2, 3, 4, 5, 6] = 1,
                 search_content: Literal[0x01, 0x02] = 0x01):
        if motor_num not in [1, 2, 3, 4, 5, 6, 7]:
            raise ValueError(f"'motor_num' Value {motor_num} out of range [1, 2, 3, 4, 5, 6, 7]")
        if search_content not in [0x01, 0x02]:
            raise ValueError(f"'search_content' Value {search_content} out of range [0x01, 0x02]")
        self.motor_num = motor_num
        self.search_content = search_content

    def __str__(self):
        return (f"ArmMsgSearchMotorMaxAngleSpdAccConfig(\n"
                f"  motor_num: {self.motor_num },\n"
                f"  search_content: {self.search_content },\n"
                f")")

    def __repr__(self):
        return self.__str__()
