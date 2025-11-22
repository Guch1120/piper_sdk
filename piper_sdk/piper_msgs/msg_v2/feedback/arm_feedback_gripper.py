#!/usr/bin/env python3
# -*-coding:utf8-*-

class ArmMsgFeedBackGripper:
    '''
    msg_v2_feedback
    
    グリッパーフィードバックメッセージ
    
    CAN ID:
        0x2A8
    
    Args:
        grippers_angle: グリッパーストローク、整数で表示。
        grippers_effort: グリッパートルク、整数で表示。
        status_code: グリッパーステータスコード、整数で表示。
    
    ビット記述:

        Byte 0: グリッパーストローク最上位, int32, 単位 0.001mm
        Byte 1: 
        Byte 2: 
        Byte 3: 
        Byte 4: グリッパートルク H, int16, 単位 0.001N/m
        Byte 5: グリッパートルク L
        Byte 6: ステータスコード, uint8
            bit[0]      電源電圧低下(0:正常 1:低下)
            bit[1]      モーター過熱(0:正常 1:過熱)
            bit[2]      ドライバ過電流(0:正常 1:過電流)
            bit[3]      ドライバ過熱(0:正常 1:過熱)
            bit[4]      センサーステータス(0:正常 1:異常)
            bit[5]      ドライバエラーステータス(0:正常 1:エラー)
            bit[6]      ドライバ有効ステータス(1:有効 0:無効)
            bit[7]      原点復帰ステータス(0:未完了 1:完了、または完了済み)
        Byte 7: 予約
    '''
    '''
    msg_v2_feedback
    
    Gripper Feedback Message

    CAN ID:
        0x2A8
    
    Args:
        grippers_angle: The stroke of the gripper, represented as an integer.
        grippers_effort: The torque of the gripper, represented as an integer.
        status_code: The status code of the gripper, represented as an integer.
    
    Bit Definitions:

        Byte Definitions:
        Byte 0: Gripper Stroke (Most Significant Byte), int32, unit: 0.001 mm
        Byte 1: Gripper Stroke (Second Most Significant Byte)
        Byte 2: Gripper Stroke (Second Least Significant Byte)
        Byte 3: Gripper Stroke (Least Significant Byte)
        Byte 4: Gripper Torque (High Byte), int16, unit: 0.001 N·m
        Byte 5: Gripper Torque (Low Byte)
        Byte 6: Status Code, uint8:
            bit[0]: Power voltage low (0: Normal, 1: Low)
            bit[1]: Motor over-temperature (0: Normal, 1: Over-temperature)
            bit[2]: Driver over-current (0: Normal, 1: Over-current)
            bit[3]: Driver over-temperature (0: Normal, 1: Over-temperature)
            bit[4]: Sensor status (0: Normal, 1: Abnormal)
            bit[5]: Driver error status (0: Normal, 1: Error)
            bit[6]: Driver enable status (1: Enabled, 0: Disabled)
            bit[7]: Zeroing status (0: Not zeroed, 1: Zeroed or previously zeroed)
        Byte 7: Reserved
    '''
    def __init__(self, 
                 grippers_angle: int = 0, 
                 grippers_effort: int = 0, 
                 status_code: int = 0):
        self.grippers_angle = grippers_angle
        self.grippers_effort = grippers_effort
        self._status_code = status_code
        self.foc_status = self.FOC_Status()
    
    class FOC_Status:
        def __init__(self):
            self.voltage_too_low  = False
            self.motor_overheating = False
            self.driver_overcurrent = False
            self.driver_overheating = False
            self.sensor_status = False
            self.driver_error_status = False
            self.driver_enable_status = False
            self.homing_status  = False
        def __str__(self): 
            return (f"    voltage_too_low : {self.voltage_too_low}\n"
                    f"    motor_overheating: {self.motor_overheating}\n"
                    f"    driver_overcurrent: {self.driver_overcurrent}\n"
                    f"    driver_overheating: {self.driver_overheating}\n"
                    f"    sensor_status: {self.sensor_status}\n"
                    f"    driver_error_status: {self.driver_error_status}\n"
                    f"    driver_enable_status: {self.driver_enable_status}\n"
                    f"    homing_status: {self.homing_status}\n"
                    )
    @property
    def status_code(self):
        return self._status_code

    @status_code.setter
    def status_code(self, value: int):
        if not (0 <= value < 2**8):
            raise ValueError("status_code must be an 8-bit integer between 0 and 255.")
        self._status_code = value
        # Update foc_status based on the status_code bits
        self.foc_status.voltage_too_low = bool(value & (1 << 0))
        self.foc_status.motor_overheating = bool(value & (1 << 1))
        self.foc_status.driver_overcurrent = bool(value & (1 << 2))
        self.foc_status.driver_overheating = bool(value & (1 << 3))
        self.foc_status.sensor_status = bool(value & (1 << 4))
        self.foc_status.driver_error_status = bool(value & (1 << 5))
        self.foc_status.driver_enable_status = bool(value & (1 << 6))
        self.foc_status.homing_status = bool(value & (1 << 7))
    
    def __str__(self):
        return (f"ArmMsgFeedBackGripper(\n"
                f"  grippers_angle: {self.grippers_angle}, {self.grippers_angle * 0.001:.3f},\n"
                f"  grippers_effort: {self.grippers_effort} \t {self.grippers_effort * 0.001:.3f},\n"
                f"  status_code: \n{self.foc_status}\n"
                f")")

    def __repr__(self):
        return self.__str__()
