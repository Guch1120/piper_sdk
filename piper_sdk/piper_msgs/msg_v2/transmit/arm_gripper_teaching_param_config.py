#!/usr/bin/env python3
# -*-coding:utf8-*-
from typing_extensions import (
    Literal,
)
class ArmMsgGripperTeachingPendantParamConfig:
    '''
    msg_v2_transmit
    
    グリッパー/ティーチングペンダントパラメータ設定コマンド(V1.5-2バージョン以降)
    
    CAN ID:
        0x47D
    
    Args:
        teaching_range_per: ティーチングペンダントストローク係数設定,[100~200]
        max_range_config: グリッパー/ティーチングペンダント最大制御ストローク制限値設定,[0,70,100]
    
    ビット記述:
    
        Byte 0: ティーチングペンダントストローク係数設定, uint8 
            ティーチングペンダントストローク係数設定---100~200,単位(%)(デフォルト100%)
            マスタースレーブアームのマスターアーム設定にのみ適用され、スレーブアームへの制御ストロークを拡大するために使用
        Byte 1: グリッパー/ティーチングペンダント最大制御ストローク制限値設定, uint8, 単位(mm)
            無効値---0
            小グリッパーは---70mm
            大グリッパーは---100mm
        Byte 2: ティーチングペンダント摩擦係数設定, uint8, 範囲[1, 10] ----- (V1.5-8バージョン以降)
        Byte 3: 予約
        Byte 4: 予約
        Byte 5: 予約
        Byte 6: 予約
        Byte 7: 予約
    '''
    '''
    msg_v2_transmit
    
    Gripper/Teaching Pendant Parameter Configuration Command(Based on version V1.5-2 and later)

    CAN ID:
        0x47D

    Args:
        teaching_range_per: Teaching pendant stroke coefficient setting.[100~200]
        max_range_config: Maximum control stroke limit setting for the gripper/teaching pendant.[0,70,100]

    Bit Description:
    
        Byte	Field	Type	Details
        Byte 0	Teaching range coefficient	uint8	- Stroke coefficient setting: 100% to 200% (default: 100%).
                    - Only applicable to the master arm in a master-slave setup to scale control range for the slave arm.
        Byte 1	Max control stroke limit	uint8	- Invalid value: 0
                    - Small gripper: 70 mm
                    - Large gripper: 100 mm
        Byte 2	Teaching pendant friction coefficient setting, `uint8`, range [1, 10].(Based on version V1.5-8 and later)
        Byte 3	Reserved	-	Reserved for future use.
        Byte 4	Reserved	-	Reserved for future use.
        Byte 5	Reserved	-	Reserved for future use.
        Byte 6	Reserved	-	Reserved for future use.
        Byte 7	Reserved	-	Reserved for future use.
    '''
    def __init__(self, 
                 teaching_range_per: int = 100, 
                 max_range_config: Literal[0, 70, 100] = 0,
                 teaching_friction: Literal[1, 2, 3, 4, 5, 6, 7, 8, 9, 10] = 1):
        if not (100 <= teaching_range_per <= 200):
            raise ValueError(f"'teaching_range_per' Value {teaching_range_per} out of range [100, 200]")
        if max_range_config not in [0, 70, 100]:
            raise ValueError(f"'max_range_config' Value {max_range_config} out of range [0,70,100]")
        if teaching_friction not in [1, 2, 3, 4, 5, 6, 7, 8, 9, 10]:
            raise ValueError(f"'teaching_friction' Value {teaching_friction} out of range [1, 2, 3, 4, 5, 6, 7, 8, 9, 10]")
        self.teaching_range_per = teaching_range_per
        self.max_range_config = max_range_config
        self.teaching_friction = teaching_friction

    def __str__(self):
        return (f"ArmMsgGripperTeachingPendantParamConfig(\n"
                f"  teaching_range_per: {self.teaching_range_per},\n"
                f"  max_range_config: {self.max_range_config},\n"
                f"  teaching_friction: {self.teaching_friction},\n"
                f")")

    def __repr__(self):
        return self.__str__()
