#!/usr/bin/env python3
# -*-coding:utf8-*-
# 注意: デモは直接実行できません。実行するにはまずpipでSDKをインストールする必要があります。
# V2バージョンSDK
# 取り付け位置を水平正装（標準）に設定します
# 側面取り付け（左/右）に設定する必要がある場合
# MotionCtrl_2(0x01,0x01,0,0,0,0x02)
# MotionCtrl_2(0x01,0x01,0,0,0,0x03)
from piper_sdk import *

if __name__ == "__main__":
    piper = C_PiperInterface_V2("can0")
    piper.ConnectPort()
    piper.MotionCtrl_2(0x01,0x01,0,0,0,0x01)  
    