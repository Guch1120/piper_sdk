#!/usr/bin/env python3
# -*-coding:utf8-*-
# 注意: デモは直接実行できません。実行するにはまずpipでSDKをインストールする必要があります。
# ロボットアームのメッセージを読み取って印刷します。piper_sdkを先にインストールする必要があります。
import time
from piper_sdk import *

# テストコード
if __name__ == "__main__":
    piper = C_PiperInterface_V2()
    piper.ConnectPort()
    while True:
        print(piper.GetArmHighSpdInfoMsgs())
        time.sleep(0.1)
    