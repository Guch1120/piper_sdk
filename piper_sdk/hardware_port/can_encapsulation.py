#!/usr/bin/env python3
# -*-coding:utf8-*-
# CANバス読み取りの二次カプセル化
import can
from can.message import Message
import time
from threading import Timer
import subprocess
from typing import (
    Callable,
    Iterator,
    Optional,
    Sequence,
    Tuple,
    Type,
    Union,
    cast,
)

class C_STD_CAN():
    '''
    基本的なCANデータフレームの送受信。内部でスレッドは作成されません。クラス外で呼び出す際に、ループ読み取り用のスレッドを作成する必要があります
    
    Args:
        channel_name: CANポート名
        bustype: CANバスタイプ、デフォルトはsocket can
        expected_bitrate: 期待されるCANバスのボーレート
        judge_flag: クラスのインスタンス化時にCANポートの判定を行うかどうか。Falseが必要な場合もあります
        auto_init: CANを自動的に初期化するかどうか、つまりcan.interface.Busをインスタンス化するかどうか
        callback_function: ReadCanMessage内のコールバック関数、関数を渡す必要があります
    '''
    '''
    Basic CAN Frame Send/Receive with Thread Creation

    When calling outside the class, a thread needs to be created to continuously read the CAN data.

    Args:
        channel_name: The name of the CAN port.
        bustype: The type of CAN bus, default is socket CAN.
        expected_bitrate: The expected bitrate for the CAN bus.
        judge_flag: Whether to check the CAN port during the instantiation of the class. In some cases, it should be set to False.
        auto_init: Whether to automatically initialize the CAN bus (i.e., instantiate can.interface.Bus).
        callback_function: The callback function in ReadCanMessage, which should be passed as a function.
    '''
    def __init__(self, 
                 channel_name:str="can0", 
                 bustype="socketcan", 
                 expected_bitrate:int=1000000,
                 judge_flag:bool=True, 
                 auto_init:bool=True,
                 callback_function: Callable = None) -> None:
        self.channel_name = channel_name
        self.bustype = bustype
        self.expected_bitrate = expected_bitrate
        self.rx_message:Optional[Message] = Message()   #メッセージ受信クラスの作成
        self.callback_function = callback_function  #受信コールバック関数
        self.bus = None
        if(judge_flag):
            self.JudgeCanInfo()
        if(auto_init):
            self.Init()#CANバスインタラクションの作成
        
    def __del__(self):
        try:
            self.bus.shutdown()  # CANバスを閉じる
            # print("CAN bus connection properly shut down.")
        except AttributeError:
            print("CAN bus connection was not properly initialized.")
        except Exception as e:
            print(f"Error occurred while shutting down CAN bus: {e}")
    
    def Init(self):
        '''CANバスの初期化
        '''
        '''Initialize the CAN bus.
        '''
        if self.bus is not None:
            return
        try:
            self.bus = can.interface.Bus(channel=self.channel_name, bustype=self.bustype)
            print(self.channel_name,"bus opened successfully.")
        except can.CanError as e:
            print(f"Failed to open CAN bus: {e}")
            self.bus = None

    def Close(self):
        '''CANバスを閉じる
        '''
        '''Close the CAN bus.
        '''
        if self.bus is not None:
            try:
                self.bus.shutdown()  # CANバスを閉じる
                # print("CAN bus connection properly shut down.")
            except AttributeError:
                print("CAN bus connection was not properly initialized.")
                return -1
            except Exception as e:
                print(f"Error occurred while shutting down CAN bus: {e}")
                return -2
            self.bus = None
            # print("CAN bus closed successfully.")
            return 1
        else:
            print("CAN bus was not open.")
            return 0
    
    def JudgeCanInfo(self):
        '''
        クラス初期化時に基本情報を検出するかどうか
        '''
        '''
        Whether to check basic information during class initialization.
        '''
        # CANポートが存在するか確認
        if not self.is_can_socket_available(self.channel_name):
            raise ValueError(f"CAN socket {self.channel_name} does not exist.")
        print(self.channel_name, " is exist")
        # CANポートがUP状態か確認
        if not self.is_can_port_up(self.channel_name):
            raise RuntimeError(f"CAN port {self.channel_name} is not UP.")
        print(self.channel_name, " is UP")
        # CANポートのボーレートを確認
        actual_bitrate = self.get_can_bitrate(self.channel_name)
        if self.expected_bitrate is not None and not (actual_bitrate == self.expected_bitrate):
            raise ValueError(f"CAN port {self.channel_name} bitrate is {actual_bitrate} bps, expected {self.expected_bitrate} bps.")
        print(self.channel_name, " bitrate is ", self.expected_bitrate)
    
    def GetBirtrate(self):
        return self.expected_bitrate

    def GetRxMessage(self) -> Message:
        return self.rx_message

    def ReadCanMessage(self):
        if self.is_can_bus_ok():
            self.rx_message = self.bus.recv()
            if self.rx_message and self.callback_function:
                self.callback_function(self.rx_message) #コールバック関数で受信した生データを処理
        else:
            print("CAN bus is not OK, skipping message read")

    def SendCanMessage(self, arbitration_id, data):
        '''can transmit

        Args:
            arbitration_id (_type_): _description_
            data (_type_): _description_
            is_extended_id_ (bool, optional): _description_. Defaults to False.
        '''
        message = can.Message(channel=self.channel_name,
                              arbitration_id=arbitration_id, 
                              data=data, 
                              dlc=8,
                              is_extended_id=False)
        if self.is_can_bus_ok():
            try:
                self.bus.send(message)
                # print(message)
                # print(f"Message sent on {self.bus.channel_info}")
            except can.CanError:
                print(can.CanError,"Message NOT sent")
        else:
            print("CAN bus is not OK, cannot send message")

    def is_can_bus_ok(self) -> bool:
        '''
        CANバスの状態が正常かどうかを確認します。
        '''
        '''
        Check whether the CAN bus status is normal.
        '''
        if isinstance(self.bus, can.BusABC):
            bus_state = self.bus.state
        else: bus_state = None
        if bus_state == can.BusState.ACTIVE:
            # print("CAN bus state: ACTIVE - Bus is functioning normally")
            return True
        elif bus_state == can.BusState.PASSIVE:
            print("CAN bus state: PASSIVE - Warning level errors are occurring")
            return False  # 必要に応じて調整可能
        elif bus_state == can.BusState.ERROR:
            print("CAN bus state: ERROR - Communication may be impaired")
            return False
        else:
            print(f"Unknown CAN bus state: {bus_state}")
            return False
    
    def is_can_socket_available(self, channel_name: str) -> bool:
        '''
        指定されたCANポートが存在するか確認します。
        '''
        '''
        Check if the given CAN port exists.
        '''
        try:
            with open(f"/sys/class/net/{channel_name}/operstate", "r") as file:
                state = file.read().strip()
                return state == "up"
        except FileNotFoundError:
            return False

    def get_can_ports(self) -> list:
        '''
        システム内のすべての利用可能なCANポートを取得します。
        '''
        '''
        Get all available CAN ports in the system.
        '''
        import os
        can_ports = []
        for item in os.listdir('/sys/class/net/'):
            if 'can' in item:
                can_ports.append(item)
        return can_ports

    def can_port_info(self, channel_name: str) -> str:
        '''
        指定されたCANポートの詳細情報（状態、タイプ、ボーレートを含む）を取得します。
        '''
        '''
        Get detailed information about the specified CAN port, including status, type, and bit rate.
        '''
        try:
            with open(f"/sys/class/net/{channel_name}/operstate", "r") as file:
                state = file.read().strip()
            with open(f"/sys/class/net/{channel_name}/type", "r") as file:
                port_type = file.read().strip()
            bitrate = self.get_can_bitrate(channel_name)
            return f"CAN port {channel_name}: State={state}, Type={port_type}, Bitrate={bitrate} bps"
        except FileNotFoundError:
            return f"CAN port {channel_name} not found."

    def is_can_port_up(self, channel_name: str) -> bool:
        '''
        CANポートがUP状態かどうかを確認します。
        '''
        '''
        Check if the CAN port is in the UP state.
        '''
        try:
            with open(f"/sys/class/net/{channel_name}/operstate", "r") as file:
                state = file.read().strip()
                return state == "up"
        except FileNotFoundError:
            return False

    def get_can_bitrate(self, channel_name: str) -> str:
        '''
        指定されたCANポートのボーレートを取得します。
        '''
        '''
        Get the bit rate of the specified CAN port.
        '''
        try:
            result = subprocess.run(['ip', '-details', 'link', 'show', channel_name],
                                    stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                    universal_newlines=True, check=True)  # Python 3.6
                                    # capture_output=True, text=True)
            output = result.stdout
            for line in output.split('\n'):
                if 'bitrate' in line:
                    return int(line.split('bitrate ')[1].split(' ')[0])
            return "Unknown"
        except Exception as e:
            print(f"Error while getting bitrate: {e}")
            return "Unknown"

## サンプルコード
# if __name__ == "__main__":
#     try:
#         can_obj = C_STD_CAN(channel_name="can0")
#         print("CAN bus initialized successfully.")
#         print(can_obj.get_can_ports())
#         print(can_obj.can_port_info("can0"))
#         print(can_obj.ReadCanMessage())
#         print(can_obj.GetRxMessage())
#     except ValueError as e:
#         print(e)
#     except Exception as e:
#         print(f"An unexpected error occurred: {e}")