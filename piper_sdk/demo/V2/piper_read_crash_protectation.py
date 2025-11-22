#!/usr/bin/env python3
# -*-coding:utf8-*-
# 注意: デモは直接実行できません。実行するにはまずpipでSDKをインストールする必要があります。
# ロボットアームの衝突保護レベルを設定して印刷します
import time
from piper_sdk import *

# テストコード
if __name__ == "__main__":
    piper = C_PiperInterface_V2("can0",False)
    piper.ConnectPort()
    # piper.CrashProtectionConfig(1,1,1,1,1,1)
    piper.CrashProtectionConfig(0,0,0,0,0,0)
    while True:
        piper.ArmParamEnquiryAndConfig(0x02, 0x00, 0x00, 0x00, 0x03)
        print(piper.GetCrashProtectionLevelFeedback())
        time.sleep(0.01)
    