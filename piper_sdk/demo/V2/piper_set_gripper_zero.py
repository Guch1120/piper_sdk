#!/usr/bin/env python3
# -*-coding:utf8-*-
# 注意: デモは直接実行できません。実行するにはまずpipでSDKをインストールする必要があります。
# グリッパーゼロ点設定デモ
import time
from piper_sdk import *

if __name__ == "__main__":
    piper = C_PiperInterface_V2("can0")
    piper.ConnectPort()
    piper.GripperCtrl(0,1000,0x00, 0)
    time.sleep(1.5)
    piper.GripperCtrl(0,1000,0x00, 0xAE)
    