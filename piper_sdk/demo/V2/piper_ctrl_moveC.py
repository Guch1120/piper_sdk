#!/usr/bin/env python3
# -*-coding:utf8-*-
# 注意: デモは直接実行できません。実行するにはまずpipでSDKをインストールする必要があります。
# Piperロボットアーム円弧モードデモ
# 注意: ロボットアームの作業スペース内に障害物がないようにしてください
import time
from piper_sdk import *

if __name__ == "__main__":
    piper = C_PiperInterface_V2("can0")
    piper.ConnectPort()
    while( not piper.EnablePiper()):
        time.sleep(0.01)
    piper.GripperCtrl(0,1000,0x01, 0)
    # MOVECモードに切り替え
    piper.MotionCtrl_2(0x01, 0x03, 30, 0x00)
    # X:135.481
    piper.EndPoseCtrl(135481,9349,161129,178756,6035,-178440)
    piper.MoveCAxisUpdateCtrl(0x01)
    time.sleep(0.001)
    piper.EndPoseCtrl(222158,128758,142126,175152,-1259,-157235)
    piper.MoveCAxisUpdateCtrl(0x02)
    time.sleep(0.001)
    piper.EndPoseCtrl(359079,3221,153470,179038,1105,179035)
    piper.MoveCAxisUpdateCtrl(0x03)
    time.sleep(0.001)
    piper.MotionCtrl_2(0x01, 0x03, 30, 0x00)
    