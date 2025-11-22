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
    print("1-----------")
    print(piper.GetRespInstruction())
    print("1-----------")
    piper.EnableArm()
    while True:
        print("------------")
        print(piper.GetRespInstruction())
        print("------------")
        if piper.GetRespInstruction().instruction_response.instruction_index == 0x71:
            # 設定コマンド0x471の応答をキャプチャした場合（ロボットアームを有効にするために送信されるコマンドIDは471）、3秒待ってからSDKに保存されている応答情報をクリアします
            # When the response to the setting command 0x471 is captured (the command ID sent to enable the robot arm is 471), 
            # wait for 3 seconds and then clear the response information saved by the SDK
            time.sleep(3)
            print("3-----------")
            piper.ClearRespSetInstruction()
            print(piper.GetRespInstruction())
            exit(0)
        time.sleep(0.005)
    