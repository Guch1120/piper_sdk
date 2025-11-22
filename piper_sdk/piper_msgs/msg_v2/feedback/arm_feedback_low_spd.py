#!/usr/bin/env python3
# -*-coding:utf8-*-
import math
from typing_extensions import (
    Literal,
)
class ArmMsgFeedbackLowSpd:
    '''
    msg_v2_feedback
    
    ドライバ情報高速フィードバック 0x6

    ノード ID:
        0x1~0x06
    CAN ID:
        0X261~0x266

    Args:
        can_id: CAN ID、現在のモーター番号を表す
        vol: 現在のドライバ電圧
        foc_temp: ドライバ温度
        motor_temp: モーター温度
        foc_status: ドライバステータスコード
        bus_current: 現在のドライバ電流、単位0.001A、1.5KG関節モーターは母線電流サンプリングなし、デフォルトで0を送信
    
    ビット記述:
    
        Byte 0:電圧上位8ビット, uint16, 現在のドライバ電圧単位: 0.1V
        Byte 1:電圧下位8ビット
        Byte 2:ドライバ温度上位8ビット, int16, 単位: 1℃
        Byte 3:ドライバ温度下位8ビット
        Byte 4:モーター温度,int8,単位: 1℃
        Byte 5:ドライバステータス,uint8
            bit[0] 電源電圧低下(0--正常; 1--低下)
            bit[1] モーター過熱(0--正常; 1--過熱)
            bit[2] ドライバ過電流(0--正常; 1--過電流)
            bit[3] ドライバ過熱(0--正常; 1--過熱)
            bit[4] 衝突保護ステータス(0--正常; 1--保護トリガー)-7.25修正、以前はセンサーステータス
            bit[5] ドライバエラーステータス(0: 正常; 1--エラー)
            bit[6] ドライバ有効ステータス(1--有効; 0--無効)
            bit[7] ストール保護ステータス(0--正常; 1--保護トリガー)-2024-7-25修正、以前は原点復帰ステータス
        Byte 6:母線電流上位8ビット,uint16,現在のドライバ電流単位: 0.001A,1.5KG関節モーターは母線電流サンプリングなし、デフォルトで0を送信
        Byte 7:母線電流下位8ビット
    '''
    '''
    msg_v2_feedback
    
    High-Speed Feedback of Drive Information 0x6

    Node ID:
        0x1~0x06

    CAN IDs:
        0x261~0x266

    Args:
        can_id: CAN ID, representing the current motor number.
        vol: Current driver voltage.
        foc_temp: Driver temperature.
        motor_temp: Motor temperature.
        foc_status: Driver status.
        bus_current: Current driver current.
    
    Bit Definitions:
    
        Byte 0: Bus Voltage (High Byte), uint16, unit: 0.1 V
        Byte 1: Bus Voltage (Low Byte)
        Byte 2: Drive Temperature (High Byte), int16, unit: 1°C
        Byte 3: Drive Temperature (Low Byte)
        Byte 4: Motor Temperature, int8, unit: 1°C
        Byte 5: Drive Status, uint8:
            bit[0]: Power voltage low (0: Normal, 1: Low)
            bit[1]: Motor over-temperature (0: Normal, 1: Over-temperature)
            bit[2]: Drive over-current (0: Normal, 1: Over-current)
            bit[3]: Drive over-temperature (0: Normal, 1: Over-temperature)
            bit[4]: Collision protection status (0: Normal, 1: Trigger protection) (Updated 7.25, previously sensor status)
            bit[5]: Drive error status (0: Normal, 1: Error)
            bit[6]: Drive enable status (1: Enabled, 0: Disabled)
            bit[7]: Stalling protection status (0: Normal, 1: Trigger protection) (Updated 7.25, previously zeroing status)
        Byte 6: Bus Current (High Byte), uint16, unit: 0.001 A, The 1.5KG joint motor has no bus current sampling and defaults to sending 0.
        Byte 7: Bus Current (Low Byte)
    '''
    def __init__(self, 
                 can_id: Literal[0x000, 0x261, 0x262, 0x263, 0x264, 0x264, 0x265, 0x266] = 0,
                 vol: int = 0, 
                 foc_temp: int = 0, 
                 motor_temp: int = 0,
                 foc_status: int = 0,
                 bus_current: int = 0,
                 ):
        if can_id not in [0x000, 0x261, 0x262, 0x263, 0x264, 0x264, 0x265, 0x266]:
            raise ValueError(f"'can_id' Value {can_id} out of range [0x000, 0x261, 0x262, 0x263, 0x264, 0x264, 0x265, 0x266]")
        self.can_id = can_id
        self.vol = vol
        self.foc_temp = foc_temp
        self.motor_temp = motor_temp
        self._foc_status_code = foc_status
        self.foc_status = self.FOC_Status()
        self.bus_current = bus_current

    class FOC_Status:
        def __init__(self):
            self.voltage_too_low  = False
            self.motor_overheating = False
            self.driver_overcurrent = False
            self.driver_overheating = False
            self.collision_status = False
            self.driver_error_status = False
            self.driver_enable_status = False
            self.stall_status  = False
        def __str__(self): 
            return (f"    voltage_too_low : {self.voltage_too_low}\n"
                    f"    motor_overheating: {self.motor_overheating}\n"
                    f"    driver_overcurrent: {self.driver_overcurrent}\n"
                    f"    driver_overheating: {self.driver_overheating}\n"
                    f"    collision_status: {self.collision_status}\n"
                    f"    driver_error_status: {self.driver_error_status}\n"
                    f"    driver_enable_status: {self.driver_enable_status}\n"
                    f"    stall_status: {self.stall_status}\n"
                    )

    @property
    def foc_status_code(self):
        return self._foc_status_code

    @foc_status_code.setter
    def foc_status_code(self, value: int):
        if not (0 <= value < 2**8):
            raise ValueError("foc_status_code must be an 8-bit integer between 0 and 255.")
        self._foc_status_code = value
        # Update foc_status based on the foc_status_code bits
        self.foc_status.voltage_too_low = bool(value & (1 << 0))
        self.foc_status.motor_overheating = bool(value & (1 << 1))
        self.foc_status.driver_overcurrent = bool(value & (1 << 2))
        self.foc_status.driver_overheating = bool(value & (1 << 3))
        self.foc_status.collision_status = bool(value & (1 << 4)) # 衝突ステータス
        self.foc_status.driver_error_status = bool(value & (1 << 5))
        self.foc_status.driver_enable_status = bool(value & (1 << 6))
        self.foc_status.stall_status = bool(value & (1 << 7)) # ストールステータス

    def __str__(self):
        return (f"ArmMsgFeedbackLowSpd(\n"
                f"  can_id: {hex(self.can_id)},\n"
                f"  vol: {self.vol}, {self.vol*0.1:.1f}V,\n"
                f"  foc_temp: {self.foc_temp }C,\n"
                f"  motor_temp: {self.motor_temp }C,\n"
                f"  foc_status: \n{self.foc_status },\n"
                f"  bus_current: {self.bus_current}, {self.bus_current*0.001:.1f}A\n"
                f")")

    def __repr__(self):
        return self.__str__()
