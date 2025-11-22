#!/usr/bin/env python3
# -*-coding:utf8-*-
# 注意: デモは直接実行できません。実行するにはまずpipでSDKをインストールする必要があります。
# V2バージョンSDK
# ロボットアーム: 全関節リミット、関節最大速度、関節加速度をデフォルト値に設定: 0x02
import time
from piper_sdk import *

if __name__ == "__main__":
    piper = C_PiperInterface_V2("can0")
    piper.ConnectPort()
    piper.ArmParamEnquiryAndConfig(0x01,0x02,0,0,0x02)
    while True:
        piper.SearchAllMotorMaxAngleSpd()
        print(piper.GetAllMotorAngleLimitMaxSpd())
        time.sleep(0.01)
    