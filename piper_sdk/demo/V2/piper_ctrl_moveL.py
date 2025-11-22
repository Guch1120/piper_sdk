#!/usr/bin/env python3
# -*-coding:utf8-*-
# 注意: デモは直接実行できません。実行するにはまずpipでSDKをインストールする必要があります。
# Piperロボットアーム直線モードデモ
# 注意: ロボットアームの作業スペース内に障害物がないようにしてください
import time
from piper_sdk import *

if __name__ == "__main__":
    piper = C_PiperInterface_V2("can0")
    piper.ConnectPort()
    while( not piper.EnablePiper()):
        time.sleep(0.01)
    
    # XOY平面上で正方形を描く
    # MOVEPモードに切り替え、初期位置に移動
    piper.MotionCtrl_2(0x01, 0x00, 100, 0x00)
    piper.EndPoseCtrl(150000, -50000, 150000, -179900, 0, -179900)
    time.sleep(2)

    # MOVELモードに切り替え
    piper.MotionCtrl_2(0x01, 0x02, 100, 0x00)
    piper.EndPoseCtrl(150000, 50000, 150000, -179900, 0, -179900)
    time.sleep(2)

    piper.MotionCtrl_2(0x01, 0x02, 100, 0x00)
    piper.EndPoseCtrl(250000, 50000, 150000, -179900, 0, -179900)
    time.sleep(2)

    piper.MotionCtrl_2(0x01, 0x02, 100, 0x00)
    piper.EndPoseCtrl(250000, -50000, 150000, -179900, 0, -179900)
    time.sleep(2)

    piper.MotionCtrl_2(0x01, 0x02, 100, 0x00)
    piper.EndPoseCtrl(150000, -50000, 150000, -179900, 0, -179900)


    