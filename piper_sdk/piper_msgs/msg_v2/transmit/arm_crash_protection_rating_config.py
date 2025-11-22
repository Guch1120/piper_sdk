#!/usr/bin/env python3
# -*-coding:utf8-*-

class ArmMsgCrashProtectionRatingConfig:
    '''
    msg_v2_transmit
    
    衝突保護レベル設定コマンド
    
    CAN ID:
        0x47A

    有効値 : 0~8

    レベル0は衝突検出なしを表す；6つの関節は独立して設定可能

    Args:
        joint_1_protection_level: 関節1の衝突レベル設定
        joint_2_protection_level: 関節2の衝突レベル設定
        joint_3_protection_level: 関節3の衝突レベル設定
        joint_4_protection_level: 関節4の衝突レベル設定
        joint_5_protection_level: 関節5の衝突レベル設定
        joint_6_protection_level: 関節6の衝突レベル設定
    
    ビット記述:
    
        Byte 0: 第1関節衝突保護レベル, uint8
        Byte 1: 第2関節衝突保護レベル, uint8
        Byte 2: 第3関節衝突保護レベル, uint8
        Byte 3: 第4関節衝突保護レベル, uint8
        Byte 4: 第5関節衝突保護レベル, uint8
        Byte 5: 第6関節衝突保護レベル, uint8
        Byte 6: 予約
        Byte 7: 予約
    '''
    '''
    msg_v2_transmit
    
    End Effector Speed/Acceleration Parameter Setting Command

    CAN ID:
        0x47A

    Valid Values: 0~8
        Level 0 indicates no collision detection.
        Collision protection levels can be set independently for the six joints.

    Args:
        joint_1_protection_level: Collision protection level for Joint 1.
        joint_2_protection_level: Collision protection level for Joint 2.
        joint_3_protection_level: Collision protection level for Joint 3.
        joint_4_protection_level: Collision protection level for Joint 4.
        joint_5_protection_level: Collision protection level for Joint 5.
        joint_6_protection_level: Collision protection level for Joint 6.

    Bit Description:

        Byte 0: Collision protection level for Joint 1, uint8.
        Byte 1: Collision protection level for Joint 2, uint8.
        Byte 2: Collision protection level for Joint 3, uint8.
        Byte 3: Collision protection level for Joint 4, uint8.
        Byte 4: Collision protection level for Joint 5, uint8.
        Byte 5: Collision protection level for Joint 6, uint8.
        Byte 6: Reserved.
        Byte 7: Reserved.
    '''
    def __init__(self, 
                 joint_1_protection_level: int = 0xFF, 
                 joint_2_protection_level: int = 0xFF, 
                 joint_3_protection_level: int = 0xFF,
                 joint_4_protection_level: int = 0xFF,
                 joint_5_protection_level: int = 0xFF,
                 joint_6_protection_level: int = 0xFF
                 ):
        self.joint_1_protection_level = joint_1_protection_level
        self.joint_2_protection_level = joint_2_protection_level
        self.joint_3_protection_level = joint_3_protection_level
        self.joint_4_protection_level = joint_4_protection_level
        self.joint_5_protection_level = joint_5_protection_level
        self.joint_6_protection_level = joint_6_protection_level

    def __str__(self):
        return (f"ArmMsgCrashProtectionRatingConfig(\n"
                f"  joint_1_protection_level: {self.joint_1_protection_level},\n"
                f"  joint_2_protection_level: {self.joint_2_protection_level},\n"
                f"  joint_3_protection_level: {self.joint_3_protection_level},\n"
                f"  joint_4_protection_level: {self.joint_4_protection_level},\n"
                f"  joint_5_protection_level: {self.joint_5_protection_level},\n"
                f"  joint_6_protection_level: {self.joint_6_protection_level}\n"
                f")")

    def __repr__(self):
        return self.__str__()
