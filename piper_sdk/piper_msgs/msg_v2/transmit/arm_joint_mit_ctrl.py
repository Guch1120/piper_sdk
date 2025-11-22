#!/usr/bin/env python3
# -*-coding:utf8-*-

class ArmMsgJointMitCtrl():
    '''
    msg_v2_transmit
    
    ロボットアーム関節MIT制御
    
    CAN ID:
        0x15A,0x15B,0x15C,0x15D,0x15E,0x15F
    
    各IDは単一の関節に対応するため、6つのIDがあります
    
    Args:
        pos_ref: 期待される目標位置を設定
        vel_ref: モーター運動速度を設定
        kp: 比例ゲイン、出力トルクに対する位置誤差の影響を制御
        kd: 微分ゲイン、出力トルクに対する速度誤差の影響を制御
        t_ref: 目標トルク参照値、モーターが加える力またはトルクを制御するために使用
        crc: 巡回冗長検査、データの完全性検証に使用
    
    ビット記述:
    
        Byte 0: Pos_ref [bit15~bit8] 上位8ビット
        Byte 1: Pos_ref [bit7~bit0]  下位8ビット
        Byte 2: Vel_ref [bit11~bit4] 下位12ビット
        Byte 3: Vel_ref [bit3~bit0], Kp [bit11~bit8]
        Byte 4: Kp [bit7~bit0],      Kp指定参照値: 10
        Byte 5: Kd [bit11~bit4]      下位12ビット,Kd指定参照値: 0.8
        Byte 6: Kd [bit3~bit0] T_ref [bit7~bit4]
        Byte 7: T_ref [bit3~bit0] CRC [bit3~bit0]
    '''
    '''
    msg_v2_transmit
    
    Mechanical Arm Joint MIT Control

    CAN IDs:
        0x15A, 0x15B, 0x15C, 0x15D, 0x15E, 0x15F

    Each ID corresponds to a single joint, thus there are six IDs.
    
    Args:
        pos_ref: Desired target position
        vel_ref: Desired motor motion speed
        kp: Proportional gain, controls the influence of position error on output torque
        kd: Derivative gain, controls the influence of velocity error on output torque
        t_ref: Target torque reference value, used to control the motor's applied force or torque
        crc: Cyclic Redundancy Check for data integrity verification
    
    Bit Description:
    
        Byte 0	Pos_ref	bit15~bit8	High 8 bits of pos_ref
        Byte 1	Pos_ref	bit7~bit0	Low 8 bits of pos_ref
        Byte 2	Vel_ref	bit11~bit4	Low 12 bits of vel_ref
        Byte 3	Vel_ref, Kp	bit3~bit0, bit11~bit8	Remaining 4 bits of vel_ref, high 4 bits of kp
        Byte 4	Kp	bit7~bit0	Low 8 bits of kp (default: 10)
        Byte 5	Kd	bit11~bit4	Low 12 bits of kd (default: 0.8)
        Byte 6	Kd, T_ref	bit3~bit0, bit7~bit4	Remaining 4 bits of kd, high 4 bits of t_ref
        Byte 7	T_ref, CRC	bit3~bit0, bit3~bit0	Low 4 bits of t_ref, low 4 bits of crc
    '''
    def __init__(self, 
                 pos_ref = 0, 
                 vel_ref = 0, 
                 kp = 10, 
                 kd = 0.8,
                 t_ref = 0, 
                 crc = 0):
        self.pos_ref = pos_ref
        self.vel_ref = vel_ref
        self.kp = kp
        self.kd = kd
        self.t_ref = t_ref
        self.crc = crc
    
    def __str__(self):
        # 角度に0.001を掛け、小数点以下3桁を保持
        mit_args = [
            ("pos_ref", self.pos_ref),
            ("vel_ref", self.vel_ref ),
            ("kp", self.kp ),
            ("kd", self.kd ),
            ("t_ref", self.t_ref ),
            ("crc", self.crc )
        ]

        # フォーマット文字列を生成、小数点以下3桁を保持
        formatted_str = "\n".join([f"{name}: {param}" for name, param in mit_args])
        
        return f"ArmMsgJointMitCtrl:\n{formatted_str}"
    
    def __repr__(self):
        return self.__str__()

class ArmMsgAllJointMitCtrl:
    '''
    msg_v2_transmit
    
    ロボットアーム関節MIT制御、全関節
    
    CAN ID:
        0x15A,0x15B,0x15C,0x15D,0x15E,0x15F
    
    各IDは単一の関節に対応するため、6つのIDがあります
    
    Args:
        pos_ref: 期待される目標位置を設定
        vel_ref: モーター運動速度を設定
        kp: 比例ゲイン、出力トルクに対する位置誤差の影響を制御
        kd: 微分ゲイン、出力トルクに対する速度誤差の影響を制御
        t_ref: 目標トルク参照値、モーターが加える力またはトルクを制御するために使用
        crc: 巡回冗長検査、データの完全性検証に使用
    
    ビット記述:
    
        Byte 0: Pos_ref [bit15~bit8] 上位8ビット
        Byte 1: Pos_ref [bit7~bit0]  下位8ビット
        Byte 2: Vel_ref [bit11~bit4] 下位12ビット
        Byte 3: Vel_ref [bit3~bit0], Kp [bit11~bit8]
        Byte 4: Kp [bit7~bit0],      Kp指定参照値: 10
        Byte 5: Kd [bit11~bit4]      下位12ビット,Kd指定参照値: 0.8
        Byte 6: Kd [bit3~bit0] T_ref [bit7~bit4]
        Byte 7: T_ref [bit3~bit0] CRC [bit3~bit0]
    '''
    '''
    msg_v2_transmit
    
    Mechanical Arm Joint MIT Control

    CAN IDs:
        0x15A, 0x15B, 0x15C, 0x15D, 0x15E, 0x15F

    Each ID corresponds to a single joint, thus there are six IDs.
    
    Args:
        pos_ref: Desired target position
        vel_ref: Desired motor motion speed
        kp: Proportional gain, controls the influence of position error on output torque
        kd: Derivative gain, controls the influence of velocity error on output torque
        t_ref: Target torque reference value, used to control the motor's applied force or torque
        crc: Cyclic Redundancy Check for data integrity verification
    
    Bit Description:
    
        Byte 0	Pos_ref	bit15~bit8	High 8 bits of pos_ref
        Byte 1	Pos_ref	bit7~bit0	Low 8 bits of pos_ref
        Byte 2	Vel_ref	bit11~bit4	Low 12 bits of vel_ref
        Byte 3	Vel_ref, Kp	bit3~bit0, bit11~bit8	Remaining 4 bits of vel_ref, high 4 bits of kp
        Byte 4	Kp	bit7~bit0	Low 8 bits of kp (default: 10)
        Byte 5	Kd	bit11~bit4	Low 12 bits of kd (default: 0.8)
        Byte 6	Kd, T_ref	bit3~bit0, bit7~bit4	Remaining 4 bits of kd, high 4 bits of t_ref
        Byte 7	T_ref, CRC	bit3~bit0, bit3~bit0	Low 4 bits of t_ref, low 4 bits of crc
    '''
    def __init__(self, 
                 m1:ArmMsgJointMitCtrl=ArmMsgJointMitCtrl(0,0,10,0.8,0,0), 
                 m2:ArmMsgJointMitCtrl=ArmMsgJointMitCtrl(0,0,10,0.8,0,0),
                 m3:ArmMsgJointMitCtrl=ArmMsgJointMitCtrl(0,0,10,0.8,0,0), 
                 m4:ArmMsgJointMitCtrl=ArmMsgJointMitCtrl(0,0,10,0.8,0,0), 
                 m5:ArmMsgJointMitCtrl=ArmMsgJointMitCtrl(0,0,10,0.8,0,0), 
                 m6:ArmMsgJointMitCtrl=ArmMsgJointMitCtrl(0,0,10,0.8,0,0)
                 ):
        self.__m = [ArmMsgJointMitCtrl(0,0,10,0.8,0,0), m1, m2, m3, m4, m5, m6]
        self.motor = [ArmMsgJointMitCtrl(0,0,10,0.8,0,0) for _ in range(7)]

    def assign(self):
        for i in range(1,7):
            if(self.__m[i].joint_motor_num != 0):
                self.motor[i] = self.__m[i]
    
    def __str__(self):
        return (f"{self.motor[1]}\n"
                f"{self.motor[2]}\n"
                f"{self.motor[3]}\n"
                f"{self.motor[4]}\n"
                f"{self.motor[5]}\n"
                f"{self.motor[6]}\n"
                f")")

    def __repr__(self):
        return self.__str__()