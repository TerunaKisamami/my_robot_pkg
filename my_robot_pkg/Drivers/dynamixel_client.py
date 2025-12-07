#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
from dynamixel_sdk import *  # dynamixel_sdk自体はLinuxでも動きます


class Dynamixel:
    def __init__(self, port, baudrate):
        """
        ROS 2 / Linux 用 Dynamixel初期化
        エラー時は getch() で待機せず、Exceptionを発生させてノードに知らせるように変更
        """

        # ********* DYNAMIXEL Model definition *********
        self.__ADDR_TORQUE_ENABLE = 64
        self.__ADDR_OPERATION_MODE = 11

        self.__ADDR_VELOCITY_LIMIT = 44
        self.__ADDR_GOAL_VELOCITY = 104
        self.__ADDR_PRESENT_VELOCITY = 128

        self.__ADDR_MAXIMUM_POSITION = 48
        self.__ADDR_MINIMUM_POSITION = 52
        self.__ADDR_GOAL_POSITION = 116
        self.__ADDR_PRESENT_POSITION = 132

        self.__BAUDRATE = baudrate
        self.__PROTOCOL_VERSION = 2.0
        self.__DEVICENAME = port

        self.__TORQUE_ENABLE = 1
        self.__TORQUE_DISABLE = 0
        self.__VELOCITY_MODE = 1
        self.__POSITION_MODE = 3
        self.__EX_POSITION_MODE = 4

        self.__portHandler = PortHandler(self.__DEVICENAME)
        self.__packetHandler = PacketHandler(self.__PROTOCOL_VERSION)

        # Open port
        if self.__portHandler.openPort():
            print(f"Succeeded to open the port: {port}")
        else:
            # msvcrt.getch() ではなく Exception を投げる
            raise Exception(f"Failed to open the port: {port}")

        # Set port baudrate
        if self.__portHandler.setBaudRate(self.__BAUDRATE):
            print(f"Succeeded to change the baudrate: {baudrate}")
        else:
            # msvcrt.getch() ではなく Exception を投げる
            raise Exception(f"Failed to change the baudrate")

    def set_mode_position(self, id):
        dxl_comm_result, dxl_error = self.__packetHandler.write1ByteTxRx(
            self.__portHandler, id, self.__ADDR_OPERATION_MODE, self.__POSITION_MODE
        )
        self._check_error(dxl_comm_result, dxl_error)

    def set_min_max_position(self, id, min_position, max_position):
        # Min
        dxl_comm_result, dxl_error = self.__packetHandler.write4ByteTxRx(
            self.__portHandler, id, self.__ADDR_MINIMUM_POSITION, min_position
        )
        self._check_error(dxl_comm_result, dxl_error)

        # Max
        dxl_comm_result, dxl_error = self.__packetHandler.write4ByteTxRx(
            self.__portHandler, id, self.__ADDR_MAXIMUM_POSITION, max_position
        )
        self._check_error(dxl_comm_result, dxl_error)

    def enable_torque(self, id):
        dxl_comm_result, dxl_error = self.__packetHandler.write1ByteTxRx(
            self.__portHandler, id, self.__ADDR_TORQUE_ENABLE, self.__TORQUE_ENABLE
        )
        self._check_error(dxl_comm_result, dxl_error)

    def disable_torque(self, id):
        dxl_comm_result, dxl_error = self.__packetHandler.write1ByteTxRx(
            self.__portHandler, id, self.__ADDR_TORQUE_ENABLE, self.__TORQUE_DISABLE
        )
        self._check_error(dxl_comm_result, dxl_error)

    def write_position(self, id, pos):
        dxl_comm_result, dxl_error = self.__packetHandler.write4ByteTxRx(
            self.__portHandler, id, self.__ADDR_GOAL_POSITION, int(pos)
        )
        self._check_error(dxl_comm_result, dxl_error)
        # print(f"[ID:{id:03d}] GoalPos:{pos:03d}") # ログがうるさければコメントアウト

    def read_position(self, id):
        dxl_present_position, dxl_comm_result, dxl_error = (
            self.__packetHandler.read4ByteTxRx(
                self.__portHandler, id, self.__ADDR_PRESENT_POSITION
            )
        )
        self._check_error(dxl_comm_result, dxl_error)
        return dxl_present_position

    def close_port(self):
        self.__portHandler.closePort()

    def _check_error(self, dxl_comm_result, dxl_error):
        """エラーチェック用ヘルパー関数"""
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.__packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.__packetHandler.getRxPacketError(dxl_error))
