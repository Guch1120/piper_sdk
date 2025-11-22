#!/usr/bin/env python3
# -*-coding:utf8-*-
from typing_extensions import (
    Literal,
)

class ArmMsgInstructionResponseConfig:
    '''
    msg_v2_transmit
    
    設定コマンド応答

    CAN ID:
        0x476

    Args:
        instruction_index: 応答コマンドインデックス
        zero_config_success_flag: ゼロ点設定成功フラグ
    
    ビット記述:
    
        Byte 0: uint8, 応答コマンドインデックス
                設定コマンドIDの最後のバイトを取得
                例：0x471設定コマンドに応答する場合、このバイトに0x71を埋める
        Byte 1: uint8, ゼロ点設定成功フラグ
                ゼロ点設定成功 : 0x01
                設定失敗/未設定: 0x00
                関節設定コマンド--N号モーターの現在位置をゼロ点に設定成功した場合のみ0x01を応答
    '''
    '''
    msg_v2_transmit
    
    Set Command Response

    CAN ID:
        0x476

    Args:
        instruction_index: Response instruction index.
        zero_config_success_flag: Whether the zero-point configuration was successful.

    Bit Description:

        Byte 0: uint8, response instruction index.
            Fill in the last byte of the set command ID.
                Example: Responding to the 0x471 set command, this byte will be 0x71.
        Byte 1: uint8, zero-point configuration success flag.
            0x01: Zero-point successfully set.
            0x00: Failed to set/Not set.
    '''
    def __init__(self,
                 instruction_index: int = 0,
                 zero_config_success_flag: Literal[0x00, 0x01] =0 ):
        if zero_config_success_flag not in [0x00, 0x01]:
            raise ValueError(f"'zero_config_success_flag' Value {zero_config_success_flag} out of range [0x01, 0x02]")
        self.instruction_index = instruction_index
        self.zero_config_success_flag = zero_config_success_flag

    def __str__(self):
        return (f"ArmMsgInstructionResponseConfig(\n"
                f"  instruction_index: {self.instruction_index },\n"
                f"  zero_config_success_flag: {self.zero_config_success_flag },\n"
                f")")

    def __repr__(self):
        return self.__str__()
