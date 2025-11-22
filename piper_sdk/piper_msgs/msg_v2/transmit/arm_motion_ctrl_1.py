#!/usr/bin/env python3
# -*-coding:utf8-*-
from typing_extensions import (
    Literal,
)
class ArmMsgMotionCtrl_1():
    '''
    msg_v2_transmit
    
    ロボットアーム運動制御コマンド1
    
    CAN ID:
        0x150
    
    Args:
        emergency_stop: 緊急停止
        track_ctrl: 軌道コマンド
        grag_teach_ctrl: ドラッグティーチングコマンド
    
    ビット記述:
    
        Byte 0: 緊急停止     uint8    0x00 無効
                                    0x01 緊急停止
                                    0x02 復帰
        Byte 1: 軌道コマンド     uint8    0x00 オフ
                                    0x01 現在の計画を一時停止
                                    0x02 現在の軌道を継続
                                    0x03 現在の軌道をクリア
                                    0x04 全ての軌道をクリア
                                    0x05 現在の計画軌道を取得
                                    0x06 実行終了
                                    0x07 軌道転送
                                    0x08 軌道転送終了
        Byte 2: ドラッグティーチングコマンド uint8     0x00 オフ
                                    0x01 ティーチング記録開始（ドラッグティーチングモードに入る）
                                    0x02 ティーチング記録終了（ドラッグティーチングモードを終了）
                                    0x03 ティーチング軌道実行（ドラッグティーチング軌道再生）
                                    0x04 実行一時停止
                                    0x05 実行再開（軌道再生再開）
                                    0x06 実行終了
                                    0x07 軌道始点へ移動
        Byte 3: 軌道インデックス    uint8     直前に転送された軌道点をN番目の軌道点としてマーク
                                    N=0~255
                                    メインコントローラ受信後、0x476 byte0 = 0x50 ;byte 2=N（詳細は0x476を参照）で応答、応答がない場合は再送が必要
        Byte 4: NameIndex_H uint16   現在の軌道パケット名インデックス、NameIndexとcrcで構成（応答0x477 byte0=03）
        Byte 5: NameIndex_L
        Byte 6: crc16_H     uint16  
        Byte 7: crc16_L
    '''
    '''
    msg_v2_transmit
    
    Robotic Arm Motion Control Command 1

    CAN ID:
        0x150

    Args:
        emergency_stop: Emergency stop command.
        track_ctrl: Trajectory control command.
        grag_teach_ctrl: Drag teach command.

    Bit Descriptions:

        Byte 0: emergency_stop: uint8, emergency stop control.
            0x00: Invalid.
            0x01: Activate emergency stop.
            0x02: Resume from emergency stop.

        Byte 1: track_ctrl: uint8, trajectory control instructions.
            0x00: Disable.
            0x01: Pause current planning.
            0x02: Resume current trajectory.
            0x03: Clear current trajectory.
            0x04: Clear all trajectories.
            0x05: Get the current planned trajectory.
            0x06: Terminate execution.
            0x07: Trajectory transfer.
            0x08: End trajectory transfer.

        Byte 2: grag_teach_ctrl: uint8, drag teach control.
            0x00: Disable.
            0x01: Start teaching record (enter drag teach mode).
            0x02: End teaching record (exit drag teach mode).
            0x03: Execute the teaching trajectory (reproduce drag teaching trajectory).
            0x04: Pause execution.
            0x05: Continue execution (resume trajectory reproduction).
            0x06: Terminate execution.
            0x07: Move to the starting point of the trajectory.

        Byte 3: trajectory_index: uint8, mark the transmitted trajectory point as the Nth trajectory point.
            N = 0~255:
            The controller responds with CAN ID: 0x476, Byte 0 = 0x50, Byte 2 = N. If no response is received, retransmission is required.

        Byte 4-5: NameIndex_H/L: uint16, current trajectory packet name index, composed of NameIndex and CRC.
            Response on CAN ID: 0x477, Byte 0 = 0x03.

        Byte 6-7: crc16_H/L: uint16, CRC checksum for validation.
    '''
    def __init__(self, 
                 emergency_stop: Literal[0x00, 0x01, 0x02] = 0, 
                 track_ctrl: Literal[0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08] = 0, 
                 grag_teach_ctrl: Literal[0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07] = 0):
        # emergency_stopが有効範囲内かチェック
        if emergency_stop not in [0x00, 0x01, 0x02]:
            raise ValueError(f"'emergency_stop' Value {emergency_stop} out of range [0x00, 0x01, 0x02]")
        
        # track_ctrlが有効範囲内かチェック
        if track_ctrl not in [0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08]:
            raise ValueError(f"'track_ctrl' Value {track_ctrl} out of range [0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08]")
        
        # grag_teach_ctrlが有効範囲内かチェック
        if grag_teach_ctrl not in [0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07]:
            raise ValueError(f"'grag_teach_ctrl' Value {grag_teach_ctrl} out of range [0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07]")
        self.emergency_stop = emergency_stop
        self.track_ctrl = track_ctrl
        self.grag_teach_ctrl = grag_teach_ctrl

    def __str__(self):
        dict_ = [
            (" emergency_stop ", self.emergency_stop),
            (" track_ctrl ", self.track_ctrl),
            (" grag_teach_ctrl ", self.grag_teach_ctrl)
        ]

        # フォーマット文字列を生成、小数点以下3桁を保持
        formatted_ = "\n".join([f"{name}: {value}" for name, value in dict_])
        
        return f"ArmMsgMotionCtrl_1:\n{formatted_}"
    
    def __repr__(self):
        return self.__str__()