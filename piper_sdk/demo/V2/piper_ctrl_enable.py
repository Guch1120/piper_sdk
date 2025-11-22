#!/usr/bin/env python3
# -*-coding:utf8-*-
# 注意: デモは直接実行できません。実行するにはまずpipでSDKをインストールする必要があります。
# ロボットアームを有効化（Enable）します
import time
from piper_sdk import *

# テストコード
if __name__ == "__main__":
    piper = C_PiperInterface_V2()
    piper.ConnectPort()
    time.sleep(0.1)
    while( not piper.EnablePiper()):
        time.sleep(0.01)
    print("有効化成功!!!!")
    