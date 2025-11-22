#!/usr/bin/env python3
# -*-coding:utf8-*-
# 注意: デモは直接実行できません。実行するにはまずpipでSDKをインストールする必要があります。
# ロボットアームのメッセージを読み取って印刷します。piper_sdkを先にインストールする必要があります。
import time
from piper_sdk import *

# テストコード
if __name__ == "__main__":
    piper = C_PiperInterface_V2("can0")
    piper.ConnectPort(True)
    count = 0
    while True:
        count += 1
        print("---------------",count)
        if(count > 200 and count < 400):
            piper.DisconnectPort()
        elif(count > 400):
            piper.ConnectPort()
        print()
        print(f"isOK: {piper.isOk()}")
        print(f"can: {piper.GetCanFps()}")
        print(f"status: {piper.GetArmStatus().Hz}")
        print(f"end_pose: {piper.GetArmEndPoseMsgs().Hz}")
        print(f"joint_states: {piper.GetArmJointMsgs().Hz}")
        print(f"gripper_msg: {piper.GetArmGripperMsgs().Hz}")
        print(f"high_spd: {piper.GetArmHighSpdInfoMsgs().Hz}")
        print(f"low_spd: {piper.GetArmLowSpdInfoMsgs().Hz}")
        print(f"joint_ctrl: {piper.GetArmJointCtrl().Hz}")
        print(f"gripper_ctrl: {piper.GetArmGripperCtrl().Hz}")
        print(f"ctrl_151: {piper.GetArmCtrlCode151().Hz}")
        print()
        time.sleep(0.01)
    