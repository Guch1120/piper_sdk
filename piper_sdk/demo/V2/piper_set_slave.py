#!/usr/bin/env python3
# -*-coding:utf8-*-
# 注意: デモは直接実行できません。実行するにはまずpipでSDKをインストールする必要があります。
# ロボットアームをスレーブアーム（従動臂）に設定します
# 注意: ロボットアームがマスターアームモードにある場合、設定コマンドを送信した後にロボットアームを再起動する必要があります
from piper_sdk import *

# テストコード
if __name__ == "__main__":
    piper = C_PiperInterface_V2()
    piper.ConnectPort()
    piper.MasterSlaveConfig(0xFC, 0, 0, 0)
    