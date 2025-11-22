#!/usr/bin/env python3
# -*-coding:utf8-*-

from abc import ABC, abstractmethod
import time
from enum import Enum, auto
import can
from can.message import Message

from typing import (
    Optional,
)
from .arm_msg_type import ArmMsgType
# feedbackサブモジュールのクラスをインポート
from .feedback import *
# transmitサブモジュールのクラスをインポート
from .transmit import *

class PiperMessage:
    '''
    msg_v2
    
    Piperロボットアームの全メッセージ、全メッセージの要約
    '''
    '''
    msg_v2
    
    Piper Robotic Arm Complete Message Summary
    '''
    def __init__(self,                 #  フィードバック
                 type_: ArmMsgType = None,
                 time_stamp: float = 0.0,
                 arm_status_msgs: ArmMsgFeedbackStatus = None,
                 arm_joint_feedback: ArmMsgFeedBackJointStates = None,
                 gripper_feedback: ArmMsgFeedBackGripper = None,
                 arm_end_pose: ArmMsgFeedBackEndPose=None,
                 arm_feedback_current_motor_angle_limit_max_spd:ArmMsgFeedbackCurrentMotorAngleLimitMaxSpd=None,
                 arm_feedback_current_end_vel_acc_param:ArmMsgFeedbackCurrentEndVelAccParam=None,
                 arm_feedback_current_motor_max_acc_limit:ArmMsgFeedbackCurrentMotorMaxAccLimit=None,
                 arm_crash_protection_rating_feedback:ArmMsgFeedbackCrashProtectionRating=None,
                 # arm_feedback_joint_vel_acc:ArmMsgFeedbackJointVelAcc=None
                 # arm_feedback_all_current_motor_angle_limit_max_spd:ArmMsgFeedbackAllCurrentMotorAngleLimitMaxSpd=None,
                 # arm_feedback_all_motor_max_acc_limit:ArmMsgFeedbackAllCurrentMotorMaxAccLimit=None,
                 arm_high_spd_feedback:ArmMsgFeedbackHighSpd=None,
                 arm_low_spd_feedback:ArmMsgFeedbackLowSpd=None,
                 arm_gripper_teaching_param_feedback:ArmMsgFeedbackGripperTeachingPendantParam=None,
                 arm_feedback_resp_set_instruction:ArmMsgFeedbackRespSetInstruction=None,
                 #  送信
                 arm_motion_ctrl_1: ArmMsgMotionCtrl_1=None,
                 arm_motion_ctrl_2: ArmMsgMotionCtrl_2=None,
                 arm_motion_ctrl_cartesian: ArmMsgMotionCtrlCartesian=None,
                 arm_joint_ctrl: ArmMsgJointCtrl=None,
                 arm_circular_ctrl: ArmMsgCircularPatternCoordNumUpdateCtrl=None,
                 arm_gripper_ctrl: ArmMsgGripperCtrl=None,
                 arm_joint_mit_ctrl: ArmMsgJointMitCtrl=None,
                 arm_ms_config: ArmMsgMasterSlaveModeConfig=None,
                 arm_motor_enable: ArmMsgMotorEnableDisableConfig=None,
                 arm_search_motor_max_angle_spd_acc_limit:ArmMsgSearchMotorMaxAngleSpdAccLimit=None,
                 arm_motor_angle_limit_max_spd_set:ArmMsgMotorAngleLimitMaxSpdSet=None,
                 arm_joint_config:ArmMsgJointConfig=None,
                 arm_set_instruction_response:ArmMsgInstructionResponseConfig=None,
                 arm_param_enquiry_and_config:ArmMsgParamEnquiryAndConfig=None,
                 arm_end_vel_acc_param_config:ArmMsgEndVelAccParamConfig=None,
                 arm_crash_protection_rating_config:ArmMsgCrashProtectionRatingConfig=None,
                 arm_gripper_teaching_param_config:ArmMsgGripperTeachingPendantParamConfig=None
                 ):
        #-------------------------------フィードバック-------------------------------------------
        # データフレームタイプの初期化
        self.type_ = type_
        # タイムスタンプ
        self.time_stamp = time_stamp
        # ロボットアームステータスメッセージの初期化
        self.arm_status_msgs = arm_status_msgs if arm_status_msgs else ArmMsgFeedbackStatus()
        # ロボットアーム関節フィードバックの初期化
        self.arm_joint_feedback = arm_joint_feedback if arm_joint_feedback else ArmMsgFeedBackJointStates()
        # グリッパーフィードバックの初期化
        self.gripper_feedback = gripper_feedback if gripper_feedback else ArmMsgFeedBackGripper()
        # エンドポーズフィードバックの初期化
        self.arm_end_pose = arm_end_pose if arm_end_pose else ArmMsgFeedBackEndPose()
        # ドライバ情報高速フィードバック
        self.arm_high_spd_feedback_1 = arm_high_spd_feedback if arm_high_spd_feedback else ArmMsgFeedbackHighSpd()
        self.arm_high_spd_feedback_2 = arm_high_spd_feedback if arm_high_spd_feedback else ArmMsgFeedbackHighSpd()
        self.arm_high_spd_feedback_3 = arm_high_spd_feedback if arm_high_spd_feedback else ArmMsgFeedbackHighSpd()
        self.arm_high_spd_feedback_4 = arm_high_spd_feedback if arm_high_spd_feedback else ArmMsgFeedbackHighSpd()
        self.arm_high_spd_feedback_5 = arm_high_spd_feedback if arm_high_spd_feedback else ArmMsgFeedbackHighSpd()
        self.arm_high_spd_feedback_6 = arm_high_spd_feedback if arm_high_spd_feedback else ArmMsgFeedbackHighSpd()
        # ドライバ情報低速フィードバック
        self.arm_low_spd_feedback_1 = arm_low_spd_feedback if arm_low_spd_feedback else ArmMsgFeedbackLowSpd()
        self.arm_low_spd_feedback_2 = arm_low_spd_feedback if arm_low_spd_feedback else ArmMsgFeedbackLowSpd()
        self.arm_low_spd_feedback_3 = arm_low_spd_feedback if arm_low_spd_feedback else ArmMsgFeedbackLowSpd()
        self.arm_low_spd_feedback_4 = arm_low_spd_feedback if arm_low_spd_feedback else ArmMsgFeedbackLowSpd()
        self.arm_low_spd_feedback_5 = arm_low_spd_feedback if arm_low_spd_feedback else ArmMsgFeedbackLowSpd()
        self.arm_low_spd_feedback_6 = arm_low_spd_feedback if arm_low_spd_feedback else ArmMsgFeedbackLowSpd()
        # グリッパー/ティーチングペンダントパラメータフィードバックコマンド
        self.arm_gripper_teaching_param_feedback = arm_gripper_teaching_param_feedback \
            if arm_gripper_teaching_param_feedback else ArmMsgFeedbackGripperTeachingPendantParam()
        #-------------------------------送信-------------------------------------------
        self.arm_motion_ctrl_1 = arm_motion_ctrl_1 if arm_motion_ctrl_1 else ArmMsgMotionCtrl_1()
        self.arm_motion_ctrl_2 = arm_motion_ctrl_2 if arm_motion_ctrl_2 else ArmMsgMotionCtrl_2()
        self.arm_motion_ctrl_cartesian = arm_motion_ctrl_cartesian if arm_motion_ctrl_cartesian else ArmMsgMotionCtrlCartesian()
        self.arm_joint_ctrl = arm_joint_ctrl if arm_joint_ctrl else ArmMsgJointCtrl()
        self.arm_circular_ctrl = arm_circular_ctrl \
            if arm_circular_ctrl else ArmMsgCircularPatternCoordNumUpdateCtrl()
        # グリッパー制御
        self.arm_gripper_ctrl = arm_gripper_ctrl if arm_gripper_ctrl else ArmMsgGripperCtrl()
        # 関節MIT制御
        self.arm_joint_mit_ctrl = arm_joint_mit_ctrl if arm_joint_mit_ctrl else ArmMsgJointMitCtrl()
        self.arm_ms_config = arm_ms_config if arm_ms_config else ArmMsgMasterSlaveModeConfig()
        # モーター有効/無効設定コマンド
        self.arm_motor_enable = arm_motor_enable if arm_motor_enable else ArmMsgMotorEnableDisableConfig()
        # モーター角度/最大速度/最大加速度制限クエリコマンド
        self.arm_search_motor_max_angle_spd_acc_limit = arm_search_motor_max_angle_spd_acc_limit \
            if arm_search_motor_max_angle_spd_acc_limit else ArmMsgSearchMotorMaxAngleSpdAccLimit()
        # 現在のモーター制限角度/最大速度をフィードバック
        self.arm_feedback_current_motor_angle_limit_max_spd = arm_feedback_current_motor_angle_limit_max_spd \
            if arm_feedback_current_motor_angle_limit_max_spd else ArmMsgFeedbackCurrentMotorAngleLimitMaxSpd()
        # モーター角度制限/最大速度設定コマンド
        self.arm_motor_angle_limit_max_spd_set = arm_motor_angle_limit_max_spd_set \
            if arm_motor_angle_limit_max_spd_set else ArmMsgMotorAngleLimitMaxSpdSet()
        # 関節設定コマンド
        self.arm_joint_config = arm_joint_config \
            if arm_joint_config else ArmMsgJointConfig()
        # 設定コマンド応答
        self.arm_set_instruction_response = arm_set_instruction_response \
            if arm_set_instruction_response else ArmMsgInstructionResponseConfig()
        # 設定コマンド応答フィードバック
        self.arm_feedback_resp_set_instruction = arm_feedback_resp_set_instruction \
            if arm_feedback_resp_set_instruction else ArmMsgFeedbackRespSetInstruction()
        # ロボットアームパラメータクエリおよび設定コマンド
        self.arm_param_enquiry_and_config = arm_param_enquiry_and_config \
            if arm_param_enquiry_and_config else ArmMsgParamEnquiryAndConfig()
        # 現在の末端速度/加速度パラメータをフィードバック
        self.arm_feedback_current_end_vel_acc_param = arm_feedback_current_end_vel_acc_param \
            if arm_feedback_current_end_vel_acc_param else ArmMsgFeedbackCurrentEndVelAccParam()
        # 末端速度/加速度パラメータ設定コマンド
        self.arm_end_vel_acc_param_config = arm_end_vel_acc_param_config \
            if arm_end_vel_acc_param_config else ArmMsgEndVelAccParamConfig()
        # 衝突保護レベル設定コマンド
        self.arm_crash_protection_rating_config = arm_crash_protection_rating_config \
            if arm_crash_protection_rating_config else ArmMsgCrashProtectionRatingConfig()
        # 衝突保護レベル設定フィードバックコマンド
        self.arm_crash_protection_rating_feedback = arm_crash_protection_rating_feedback \
            if arm_crash_protection_rating_feedback else ArmMsgFeedbackCrashProtectionRating()
        # 現在のモーター最大加速度制限をフィードバック
        self.arm_feedback_current_motor_max_acc_limit = arm_feedback_current_motor_max_acc_limit \
            if arm_feedback_current_motor_max_acc_limit else ArmMsgFeedbackCurrentMotorMaxAccLimit()
        # グリッパー/ティーチングペンダントパラメータ設定コマンド
        self.arm_gripper_teaching_param_config = arm_gripper_teaching_param_config \
            if arm_gripper_teaching_param_config else ArmMsgGripperTeachingPendantParamConfig()
        # 各関節の現在の末端速度/加速度をフィードバック
        # self.arm_feedback_joint_vel_acc = arm_feedback_joint_vel_acc \
        #     if arm_feedback_joint_vel_acc else ArmMsgFeedbackJointVelAcc()
        # 全モーターの現在の制限角度/最大速度
        # self.arm_feedback_all_current_motor_angle_limit_max_spd = arm_feedback_all_current_motor_angle_limit_max_spd \
        #     if arm_feedback_all_current_motor_angle_limit_max_spd else ArmMsgFeedbackAllCurrentMotorAngleLimitMaxSpd()
        # # 全モーターの最大加速度制限
        # self.arm_feedback_all_motor_max_acc_limit = arm_feedback_all_motor_max_acc_limit \
        #     if arm_feedback_all_motor_max_acc_limit else ArmMsgFeedbackAllCurrentMotorMaxAccLimit()
        self.firmware_data = bytearray()

    def __str__(self):
        # feedback
        if(self.type_ == ArmMsgType.PiperMsgStatusFeedback):
            return (f"Type: {self.type_}\n"f"Arm Status: {self.arm_status_msgs}\n")
        elif(self.type_ == ArmMsgType.PiperMsgJointFeedBack_12):
            return (f"Type: {self.type_}\n"f"Joint Feed: {self.arm_joint_feedback}\n")
        elif(self.type_ == ArmMsgType.PiperMsgJointFeedBack_34):
            return (f"Type: {self.type_}\n"f"Joint Feed: {self.arm_joint_feedback}\n")
        elif(self.type_ == ArmMsgType.PiperMsgJointFeedBack_56):
            return (f"Type: {self.type_}\n"f"Joint Feed: {self.arm_joint_feedback}\n")
        elif(self.type_ == ArmMsgType.PiperMsgGripperFeedBack):
            return (f"Type: {self.type_}\n"f"Gripper Feed: {self.gripper_feedback}\n")
        elif(self.type_ == ArmMsgType.PiperMsgEndPoseFeedback_1):
            return (f"Type: {self.type_}\n"f"End Pose Feed: {self.arm_end_pose}\n")
        elif(self.type_ == ArmMsgType.PiperMsgEndPoseFeedback_2):
            return (f"Type: {self.type_}\n"f"End Pose Feed: {self.arm_end_pose}\n")
        elif(self.type_ == ArmMsgType.PiperMsgEndPoseFeedback_3):
            return (f"Type: {self.type_}\n"f"End Pose Feed: {self.arm_end_pose}\n")
        elif(self.type_ == ArmMsgType.PiperMsgHighSpdFeed_1):
            return (f"Type: {self.type_}\n"f"High Spd Feedback: {self.arm_high_spd_feedback_1}\n")
        elif(self.type_ == ArmMsgType.PiperMsgHighSpdFeed_2):
            return (f"Type: {self.type_}\n"f"High Spd Feedback: {self.arm_high_spd_feedback_2}\n")
        elif(self.type_ == ArmMsgType.PiperMsgHighSpdFeed_3):
            return (f"Type: {self.type_}\n"f"High Spd Feedback: {self.arm_high_spd_feedback_3}\n")
        elif(self.type_ == ArmMsgType.PiperMsgHighSpdFeed_4):
            return (f"Type: {self.type_}\n"f"High Spd Feedback: {self.arm_high_spd_feedback_4}\n")
        elif(self.type_ == ArmMsgType.PiperMsgHighSpdFeed_5):
            return (f"Type: {self.type_}\n"f"High Spd Feedback: {self.arm_high_spd_feedback_5}\n")
        elif(self.type_ == ArmMsgType.PiperMsgHighSpdFeed_6):
            return (f"Type: {self.type_}\n"f"High Spd Feedback: {self.arm_high_spd_feedback_6}\n")
        elif(self.type_ == ArmMsgType.PiperMsgLowSpdFeed_1):
            return (f"Type: {self.type_}\n"f"Low Spd Feedback: {self.arm_low_spd_feedback_1}\n")
        elif(self.type_ == ArmMsgType.PiperMsgLowSpdFeed_2):
            return (f"Type: {self.type_}\n"f"Low Spd Feedback: {self.arm_low_spd_feedback_2}\n")
        elif(self.type_ == ArmMsgType.PiperMsgLowSpdFeed_3):
            return (f"Type: {self.type_}\n"f"Low Spd Feedback: {self.arm_low_spd_feedback_3}\n")
        elif(self.type_ == ArmMsgType.PiperMsgLowSpdFeed_4):
            return (f"Type: {self.type_}\n"f"Low Spd Feedback: {self.arm_low_spd_feedback_4}\n")
        elif(self.type_ == ArmMsgType.PiperMsgLowSpdFeed_5):
            return (f"Type: {self.type_}\n"f"Low Spd Feedback: {self.arm_low_spd_feedback_5}\n")
        elif(self.type_ == ArmMsgType.PiperMsgLowSpdFeed_6):
            return (f"Type: {self.type_}\n"f"Low Spd Feedback: {self.arm_low_spd_feedback_6}\n")
        # 送信,transmit
        elif(self.type_ == ArmMsgType.PiperMsgMotionCtrl_1):
            return (f"Type: {self.type_}\n"f"PiperMsgMotionCtrl_1: {self.arm_motion_ctrl_1}\n")
        elif(self.type_ == ArmMsgType.PiperMsgMotionCtrl_2):
            return (f"Type: {self.type_}\n"f"PiperMsgMotionCtrl_2: {self.arm_motion_ctrl_2}\n")
        elif(self.type_ == ArmMsgType.PiperMsgMotionCtrlCartesian_1):
            return (f"Type: {self.type_}\n"f"ArmMsgMotionCtrlCartesian: {self.arm_motion_ctrl_cartesian}\n")
        elif(self.type_ == ArmMsgType.PiperMsgMotionCtrlCartesian_2):
            return (f"Type: {self.type_}\n"f"ArmMsgMotionCtrlCartesian: {self.arm_motion_ctrl_cartesian}\n")
        elif(self.type_ == ArmMsgType.PiperMsgMotionCtrlCartesian_3):
            return (f"Type: {self.type_}\n"f"ArmMsgMotionCtrlCartesian: {self.arm_motion_ctrl_cartesian}\n")
        elif(self.type_ == ArmMsgType.PiperMsgJointCtrl_12):
            return (f"Type: {self.type_}\n"f"ArmMsgJointCtrl: {self.arm_joint_ctrl}\n")
        elif(self.type_ == ArmMsgType.PiperMsgJointCtrl_34):
            return (f"Type: {self.type_}\n"f"ArmMsgJointCtrl: {self.arm_joint_ctrl}\n")
        elif(self.type_ == ArmMsgType.PiperMsgJointCtrl_56):
            return (f"Type: {self.type_}\n"f"ArmMsgJointCtrl: {self.arm_joint_ctrl}\n")
        elif(self.type_ == ArmMsgType.PiperMsgGripperCtrl):
            return (f"Type: {self.type_}\n"f"PiperMsgGripperCtrl: {self.arm_gripper_ctrl}\n")
        else:
            return (f"Type: {self.type_}\n")

    def __repr__(self):
        return self.__str__()