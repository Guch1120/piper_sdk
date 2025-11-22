#!/usr/bin/env python3
# -*-coding:utf8-*-
from typing_extensions import (
    Literal,
)
class ArmMsgMotionCtrl_2():
    '''
    msg_v2_transmit
    
    ロボットアーム運動制御コマンド2

    CAN ID:
        0x151
    
    Args:
        ctrl_mode: 制御モード
        move_mode: MOVEモード
        move_spd_rate_ctrl: 運動速度パーセンテージ
        mit_mode: MITモード
        residence_time: オフライン軌道点滞留時間
        installation_pos: 設置位置
    
    ビット記述:
    
        Byte 0: 制御モード     uint8
                0x00 待機モード
                0x01 CANコマンド制御モード
                0x03 イーサネット制御モード
                0x04 Wi-Fi制御モード
                0x07 オフライン軌道モード
        Byte 1: MOVEモード     uint8    
                0x00 MOVE P
                0x01 MOVE J
                0x02 MOVE L
                0x03 MOVE C
                0x04 MOVE M ---V1.5-2バージョン以降
                0x05 MOVE CPV ---V1.8-1バージョン以降
        Byte 2: 運動速度パーセンテージ uint8    0~100
        Byte 3: MITモード      uint8   
                0x00 位置速度モード
                0xAD MITモード
                0xFF 無効
        Byte 4: オフライン軌道点滞留時間 uint8 0~254 ,単位: s;255:軌道終了
        Byte 5: 設置位置 uint8 配線が後ろ向きであることに注意 ---V1.5-2バージョン以降
                0x00 無効値
                0x01 水平正立
                0x02 左側面取り付け
                0x03 右側面取り付け
    '''
    '''
    msg_v2_transmit
    
    Robotic Arm Motion Control Command 2

    CAN ID:
        0x151
    
    Args:
        ctrl_mode: Control mode.
        move_mode: MOVE mode.
        move_spd_rate_ctrl: Movement speed as a percentage.
        mit_mode: MIT mode.
        residence_time: Hold time at offline trajectory points.

    Bit Descriptions:

        Byte 0 control_mode: uint8, control mode selection.
            0x00: Standby mode.
            0x01: CAN command control mode.
            0x03: Ethernet control mode.
            0x04: Wi-Fi control mode.
            0x07: Offline trajectory mode.

        Byte 1 move_mode: uint8, movement mode selection.
            0x00: MOVE P (Position).
            0x01: MOVE J (Joint).
            0x02: MOVE L (Linear).
            0x03: MOVE C (Circular).
            0x04: MOVE M (MIT) ---- Based on version V1.5-2 and later
            0x05: MOVE CPV ---- Based on version V1.8-1 and later

        Byte 2 speed_percentage: uint8, movement speed as a percentage.
            Range: 0~100.

        Byte 3 mit_mode: uint8, motion control mode.
            0x00: Position-speed mode.
            0xAD: MIT mode.
            0xFF: Invalid.

        Byte 4 offline_trajectory_hold_time: uint8, duration to hold at offline trajectory points.
            Range: 0~255, unit: seconds.
        
        Byte 5: Installation Position (uint8) - Note: Wiring should face backward ---- Based on version V1.5-2 and later
                0x00: Invalid value
                0x01: Horizontal upright
                0x02: Left-side mount
                0x03: Right-side mount
    '''
    def __init__(self, 
                 ctrl_mode: Literal[0x00, 0x01, 0x03, 0x04, 0x07] = 0x01, 
                 move_mode: Literal[0x00, 0x01, 0x02, 0x03, 0x04, 0x05] = 0x01, 
                 move_spd_rate_ctrl: int = 50,
                 mit_mode: Literal[0x00, 0xAD, 0xFF] = 0x00,
                 residence_time: int = 0,
                 installation_pos: Literal[0x00, 0x01, 0x02, 0x03] = 0x00):
        # 有効範囲内かチェック
        if ctrl_mode not in [0x00, 0x01, 0x03, 0x04, 0x07]:
            raise ValueError(f"'ctrl_mode' Value {ctrl_mode} out of range [0x00, 0x01, 0x02, 0x03, 0x04, 0x07]")
        if move_mode not in [0x00, 0x01, 0x02, 0x03, 0x04, 0x05]:
            raise ValueError(f"'move_mode' Value {move_mode} out of range [0x00, 0x01, 0x02, 0x03, 0x04, 0x05]")
        if not (0 <= move_spd_rate_ctrl <= 100):
            raise ValueError(f"'move_spd_rate_ctrl' Value {move_spd_rate_ctrl} out of range 0-100")
        if mit_mode not in [0x00, 0xAD, 0xFF]:
            raise ValueError(f"'mit_mode' Value {mit_mode} out of range [0x00, 0xAD, 0xFF]")
        if not (0 <= residence_time <= 255):
            raise ValueError(f"'residence_time' Value {residence_time} out of range 0-255")
        if installation_pos not in [0x00, 0x01, 0x02, 0x03]:
            raise ValueError(f"'installation_pos' Value {installation_pos} out of range [0x00, 0x01, 0x02, 0x03]")
        self.ctrl_mode = ctrl_mode
        self.move_mode = move_mode
        self.move_spd_rate_ctrl = move_spd_rate_ctrl
        self.mit_mode = mit_mode
        self.residence_time = residence_time
        self.installation_pos = installation_pos

    def __str__(self):
        dict_ = [
            (" ctrl_mode ", self.ctrl_mode),
            (" move_mode ", self.move_mode),
            (" move_spd_rate_ctrl ", self.move_spd_rate_ctrl),
            (" mit_mode ", self.mit_mode),
            (" residence_time ", self.residence_time),
            (" installation_pos ", self.installation_pos),
        ]

        # フォーマット文字列を生成、小数点以下3桁を保持
        formatted_ = "\n".join([f"{name}: {value}" for name, value in dict_])
        
        return f"ArmMsgMotionCtrl_2:\n{formatted_}"
    
    def __repr__(self):
        return self.__str__()