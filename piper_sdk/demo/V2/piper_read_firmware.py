#!/usr/bin/env python3
# -*-coding:utf8-*-
# 注意: デモは直接実行できません。実行するにはまずpipでSDKをインストールする必要があります。
# ロボットアームのファームウェアバージョンを読み取って印刷します
import time
from piper_sdk import *

# テストコード
if __name__ == "__main__":
    piper = C_PiperInterface_V2()
    piper.ConnectPort()
    time.sleep(0.03) # ファームウェアのフィードバックフレームを読み取るのに時間が必要です。そうしないと、-0x4AFが返されます。
    print(piper.GetPiperFirmwareVersion())
    # while True:
    #     piper.SearchPiperFirmwareVersion()
    #     time.sleep(0.025)
    #     print(piper.GetPiperFirmwareVersion())
    