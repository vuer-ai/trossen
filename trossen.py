from DynamixelSDK.python.src import dynamixel_sdk as Dynamixel
from enum import Enum

from config import itemWriteMultiple
from singleton import singleton

from typing import List, Tuple

import time
from DynamixelSDK.python.src.dynamixel_sdk import PortHandler


# class DXL_CONFIG:
#     ID: int
#
#     # Drive Mode
#     ## Bit 3: Torque on by Goal Update
#     ## Bit 2: Time based Profile
#     ## Bit 0: Reverse Mode
#     DRIVE_MODE: int
#
#     # Shadow ID
#     ## To follow another servo, set to their ID (Between 0-251)
#     ## To disable, set to 255
#     SHADOW_ID: int  # Between 0-251 for following. 255 for disable.
#
#     # Minimum Position Limit
#     ## Between 0-4095
#     MIN_POS_LIMIT: int
#
#     # Maximium Position Limit
#     ## Between 0-4095
#     MAX_POS_LIMIT: int  # Between 0-4095
#
#     # Operating Mode
#     ## 0: Current Control
#     ## 1: Velocity Control
#     ## 3: Position Control
#     ## 4: Extended Position Control
#     ## 5: Current-based Position Control
#     ## 16: PWM Control
#     OPERATING_MODE: int
#
#     # PID Settings
#     ## Position P gain
#     POS_P_GAIN: int
#     ## Position I gain
#     POS_I_GAIN: int
#     ## Position D gain
#     POS_D_GAIN: int
#
#     # Additional configurations viewable in dynamixel wizard
#     #
#     # all_ids = [1,2,3,4,5,6,7]
#     # drive_modes = [4,4,5,5,5,4,4]
#     # shadow_ids = [255,255,2,255,255,255,255]
#     # min_position_limits = [0,0,0,0,0,0,0]
#     # max_position_limits = [4095,4095,4095,4095,4095,4095,4095]
#     # operating_modes = [3,3,3,3,3,3,16]
#     # position_p_gains = [800, 1500, 2000, 1000, 640]
#     #
#     # ## WX200 ARM 'Command' IDs
#     # arm_ids = [1,2,4,5,6]
#     # gripper_id = 7
#     # shoulder_master_id = 2
#     # shoulder_slave_id = 3

class DXL_ID(Enum):
    BASE_ROTATE = 1
    BASE_PIVOT_1 = 2
    BASE_PIVOT_2 = 3
    ELBOW_PIVOT_1 = 4
    ELBOW_PIVOT_2 = 5
    ELBOW_ROTATE = 6
    WRIST_PIVOT = 7
    WRIST_ROTATE = 8
    END_EFFECTOR = 9

    def __call__(self):
        return self.value


class DXL_CONSTANTS:
    ADDR_DRIVE_MODE = 10
    ADDR_OPERATING_MODE = 11
    ADDR_SECONDARY_ID = 12
    ADDR_HOMING_OFFSET = 20
    ADDR_MAX_POS_LIMIT = 48
    ADDR_MIN_POS_LIMIT = 52
    ADDR_TORQUE_ENABLE = 64
    ADDR_LED = 65
    ADDR_VELOCITY_I_GAIN = 76
    ADDR_VELOCITY_P_GAIN = 78
    ADDR_POSITION_D_GAIN = 80
    ADDR_POSITION_I_GAIN = 82
    ADDR_POSITION_P_GAIN = 84
    ADDR_GOAL_PWM = 100
    ADDR_PROFILE_ACCELERATION = 108
    ADDR_PROFILE_VELOCITY = 112
    ADDR_GOAL_POSITION = 116
    ADDR_PRESENT_LOAD = 126
    ADDR_PRESENT_POSITION = 132
    ADDR_PRESENT_TEMP = 146

    LEN_DRIVE_MODE = 1
    LEN_OPERATING_MODE = 1
    LEN_SECONDARY_ID = 1
    LEN_HOMING_OFFSET = 4
    LEN_MAX_POS_LIMIT = 4
    LEN_MIN_POS_LIMIT = 4
    LEN_TORQUE_ENABLE = 1
    LEN_LED = 1
    LEN_VELOCITY_I_GAIN = 2
    LEN_VELOCITY_P_GAIN = 2
    LEN_POSITION_D_GAIN = 2
    LEN_POSITION_I_GAIN = 2
    LEN_POSITION_P_GAIN = 2
    LEN_GOAL_PWM = 2
    LEN_PROFILE_ACCELERATION = 4
    LEN_PROFILE_VELOCITY = 4
    LEN_GOAL_POSITION = 4
    LEN_PRESENT_LOAD = 2
    LEN_PRESENT_POSITION = 4
    LEN_PRESENT_TEMP = 1

    LED_OFF = 0x00
    LED_ON = 0x01

@singleton
class Trossen:
    ## WX200 ARM EEPROM CONFIG    shoulder_master_id = 2
    all_ids = [1, 2, 3, 4, 5, 6, 7, 8, 9]
    drive_modes = [4, 4, 5, 4, 5, 4, 5, 4, 0]
    shadow_ids = [255, 255, 2, 255, 4, 255, 255, 255, 255]
    min_position_limits = [0, 820, 820, 650, 650, 0, 910, 0, 1500]
    max_position_limits = [4095, 3345, 3345, 3095, 3095, 4095, 3445, 4095, 2950]
    operating_modes = [3, 3, 3, 3, 3, 3, 3, 3, 3]

    velocity_i_gains = [1920, 1920, 1920, 1920, 1920, 1920, 1920, 1000, 1000]
    velocity_p_gains = [100, 100, 100, 100, 100, 100, 100, 100, 100]

    position_d_gains = [0, 0, 0, 0, 0, 0, 0, 3600, 3600]
    position_i_gains = [0, 0, 0, 0, 0, 0, 0, 0, 0]
    position_p_gains = [800, 800, 800, 800, 800, 800, 800, 640, 640]

    home_position = [2038, 1028, 1028, 3112, 3112, 2040, 950, 2045, 2860]

    # ## WX200 ARM 'Command' IDs
    # arm_ids = [1, 2, 4, 5, 6]
    # gripper_id = 7
    shoulder_master_id = DXL_ID.BASE_PIVOT_1()
    shoulder_slave_id = DXL_ID.BASE_PIVOT_2()
    elbow_master_id = DXL_ID.ELBOW_PIVOT_1()
    elbow_slave_id = DXL_ID.ELBOW_PIVOT_2()

    def __init__(self, port: str, baudrate: int):
        self.portHandler = Dynamixel.PortHandler(port)
        self.packetHandler = Dynamixel.PacketHandler(2.0)

        if self.portHandler.openPort():
            print("Successfully opened the port at %s!" % port)
        else:
            print("Failed to open the port at %s!", port)
            raise Exception("Failed to open the port at %s!" % port)

        if self.portHandler.setBaudRate(baudrate):
            print("Succeeded to change the baudrate to %d bps!" % baudrate)
        else:
            print("Failed to change the baudrate to %d bps!" % baudrate)
            raise Exception("Failed to change the baudrate to %d bps!" % baudrate)

        self.groupSyncWrite = Dynamixel.GroupSyncWrite(self.portHandler, self.packetHandler,
                                                       DXL_CONSTANTS.ADDR_GOAL_POSITION,
                                                       DXL_CONSTANTS.LEN_GOAL_POSITION)
        self.groupSyncRead = Dynamixel.GroupSyncRead(self.portHandler, self.packetHandler,
                                                     DXL_CONSTANTS.ADDR_PRESENT_POSITION,
                                                     DXL_CONSTANTS.LEN_PRESENT_POSITION)

        ## Initialize register values
        self.itemWriteMultiple(self.all_ids, DXL_CONSTANTS.ADDR_TORQUE_ENABLE, 0, DXL_CONSTANTS.LEN_TORQUE_ENABLE)
        self.itemWriteMultiple(self.all_ids, DXL_CONSTANTS.ADDR_DRIVE_MODE, self.drive_modes,
                               DXL_CONSTANTS.LEN_DRIVE_MODE)
        self.itemWriteMultiple(self.all_ids, DXL_CONSTANTS.ADDR_SECONDARY_ID, self.shadow_ids,
                               DXL_CONSTANTS.LEN_SECONDARY_ID)
        self.itemWriteMultiple(self.all_ids, DXL_CONSTANTS.ADDR_MIN_POS_LIMIT, self.min_position_limits,
                               DXL_CONSTANTS.LEN_MIN_POS_LIMIT)
        self.itemWriteMultiple(self.all_ids, DXL_CONSTANTS.ADDR_MAX_POS_LIMIT, self.max_position_limits,
                               DXL_CONSTANTS.LEN_MAX_POS_LIMIT)
        self.itemWriteMultiple(self.all_ids, DXL_CONSTANTS.ADDR_OPERATING_MODE, self.operating_modes,
                               DXL_CONSTANTS.LEN_OPERATING_MODE)
        self.itemWriteMultiple(self.all_ids, DXL_CONSTANTS.ADDR_POSITION_P_GAIN, self.position_p_gains,
                               DXL_CONSTANTS.LEN_POSITION_P_GAIN)
        self.itemWriteMultiple(self.all_ids, DXL_CONSTANTS.ADDR_POSITION_I_GAIN, self.position_i_gains,
                               DXL_CONSTANTS.LEN_POSITION_I_GAIN)
        self.itemWriteMultiple(self.all_ids, DXL_CONSTANTS.ADDR_POSITION_D_GAIN, self.position_d_gains,
                               DXL_CONSTANTS.LEN_POSITION_D_GAIN)
        self.itemWriteMultiple(self.all_ids, DXL_CONSTANTS.ADDR_VELOCITY_I_GAIN, self.velocity_i_gains,
                               DXL_CONSTANTS.LEN_VELOCITY_I_GAIN)
        self.itemWriteMultiple(self.all_ids, DXL_CONSTANTS.ADDR_VELOCITY_P_GAIN, self.velocity_p_gains,
                               DXL_CONSTANTS.LEN_VELOCITY_P_GAIN)

        # self.itemWriteMultiple(self.all_ids, DXL_CONSTANTS.ADDR_PROFILE_VELOCITY, 1500,
        #                        DXL_CONSTANTS.LEN_PROFILE_VELOCITY)
        # self.itemWriteMultiple(self.all_ids, DXL_CONSTANTS.ADDR_PROFILE_ACCELERATION, 750,
        #                        DXL_CONSTANTS.LEN_PROFILE_ACCELERATION)

        self.calibrateDualJoint(self.shoulder_master_id, self.shoulder_slave_id)
        self.calibrateDualJoint(self.elbow_master_id, self.elbow_slave_id)

        self.itemWriteMultiple(self.all_ids, DXL_CONSTANTS.ADDR_TORQUE_ENABLE, 1, DXL_CONSTANTS.LEN_TORQUE_ENABLE)

    def torqueDisable(self):
        self.itemWriteMultiple(self.all_ids, DXL_CONSTANTS.ADDR_TORQUE_ENABLE, 0, DXL_CONSTANTS.LEN_TORQUE_ENABLE)

    def torqueEnable(self):
        self.itemWriteMultiple(self.all_ids, DXL_CONSTANTS.ADDR_TORQUE_ENABLE, 1, DXL_CONSTANTS.LEN_TORQUE_ENABLE)

    # Maps inputted
    def setEndEffectorPos(self, position: int):
        pos_min_end_effector = self.min_position_limits[DXL_ID.END_EFFECTOR()-1]
        pos_max_end_effector = self.max_position_limits[DXL_ID.END_EFFECTOR()-1]

        position = max(min(position, 4095), 0)

        # Apply the mapping formula
        mapped_pos = int(pos_min_end_effector + (position - 0) * (pos_max_end_effector - pos_min_end_effector) / (4095 - 0))
        print(f"End Effector value {position} mapped to {mapped_pos}")
        # return new_min + (value - old_min) * (new_max - new_min) / (old_max - old_min)

        self.itemWrite(DXL_ID.END_EFFECTOR(), DXL_CONSTANTS.ADDR_GOAL_POSITION, mapped_pos, DXL_CONSTANTS.LEN_GOAL_POSITION)

    def set_joint_position(self, joint_id, position):
        self.itemWrite(joint_id, DXL_CONSTANTS.ADDR_GOAL_POSITION, position, DXL_CONSTANTS.LEN_GOAL_POSITION)

    def close(self):
        self.torqueEnable()
        self.portHandler.closePort()
        print("Port closed!")
        return
    # def __del__ (self):
    #     self.portHandler.closePort()
    #     print("Port closed!")
    #     return

    def flash_leds(self):
        for i in range(10):
            self.itemWriteMultiple(self.all_ids, DXL_CONSTANTS.ADDR_LED, DXL_CONSTANTS.LED_OFF, DXL_CONSTANTS.LEN_LED)
            time.sleep(.5)
            self.itemWriteMultiple(self.all_ids, DXL_CONSTANTS.ADDR_LED, DXL_CONSTANTS.LED_ON, DXL_CONSTANTS.LEN_LED)
            time.sleep(.5)

    def positions(self) -> Tuple[List[int], bool]:
        return self.syncRead(self.groupSyncRead, self.all_ids, DXL_CONSTANTS.ADDR_PRESENT_POSITION,
                             DXL_CONSTANTS.LEN_PRESENT_POSITION)

    def home(self):
        # WRITE LATER
        return

    ### @brief Helper function to check error messages and print them if need be
    ### @param dxl_comm_result - if nonzero, there is an error in the communication
    ### @param dxl_error - if nonzero, there is an error in something related to the data
    ### @return <bool> - true if no error occurred; false otherwise
    def checkError(self, dxl_comm_result, dxl_error):
        if dxl_comm_result != Dynamixel.COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            return False
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
            return False
        return True

    ### @brief Writes data to the specified register for a given motor
    ### @param id - Dynamixel ID to write data to
    ### @param address - register number
    ### @param data - data to write
    ### @param length - size of register in bytes
    ### @return <bool> - true if data was successfully written; false otherwise
    def itemWrite(self, id, address, data, length):
        if length == 1:
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, id, address, data)
        elif length == 2:
            dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, id, address, data)
        elif length == 4:
            dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, id, address, data)
        else:
            print("Invalid data length...")
            return False
        return self.checkError(dxl_comm_result, dxl_error)

    ### @brief Sequentially writes the same data to the specified register for a group of motors
    ### @param ids - vector of IDs to write data to
    ### @param address - register number
    ### @param data - data to write (could be a list [index matches index in 'ids' list] or a single value [for all ids])
    ### @param length - size of register in bytes
    ### @return <bool> - true if data was successfully written; false otherwise
    def itemWriteMultiple(self, ids, address, data, length):
        if type(data) != list:
            for id in ids:
                success = self.itemWrite(id, address, data, length)
                if success != True:
                    return False
        else:
            for id, dat in zip(ids, data):
                success = self.itemWrite(id, address, dat, length)
                if success != True:
                    return False
        return True

    ### @brief Reads data from the specified register for a given motor
    ### @param id - Dynamixel ID to read data from
    ### @param address - register number
    ### @param length - size of register in bytes
    ### @return state - variable to store the requested data
    ### @return <bool> - true if data was successfully retrieved; false otherwise
    ### @details - DynamixelSDK uses 2's complement so we need to check to see if 'state' should be negative (hex numbers)
    def itemRead(self, id, address, length):
        if length == 1:
            state, dxl_comm_result, dxl_error = self.packetHandler.read1ByteTxRx(self.portHandler, id, address)
        elif length == 2:
            state, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, id, address)
            if state > 0x7fff:
                state = state - 65536
        elif length == 4:
            state, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler, id, address)
            if state > 0x7fffffff:
                state = state - 4294967296
        else:
            print("Invalid data length...")
            return 0, False
        return state, self.checkError(dxl_comm_result, dxl_error)

    ### @brief Sequentially reads data from the specified register for a group of motors
    ### @param id - vector of Dynamixel IDs to read data from
    ### @param address - register number
    ### @param length - size of register in bytes
    ### @return states - list to store the requested data
    ### @return <bool> - true if data was successfully retrieved; false otherwise
    def itemReadMultiple(self, ids, address, length):
        states = []
        for id in ids:
            state, success = self.itemRead(id, address, length)
            if success != True:
                return [], False
            states.append(state)
        return states, True

    ### @brief Writes data to a group of motors synchronously
    ### @param groupSyncWrite - groupSyncWrite object
    ### @param ids - list of Dynamixel IDs to write to
    ### @param commands - list of commands to write to each respective motor
    ### @param length - size of register in bytes
    ### @return <bool> - true if data was successfully written; false otherwise
    def syncWrite(self, groupSyncWrite, ids, commands, length):
        groupSyncWrite.clearParam()
        for id, cmd in zip(ids, commands):
            param = []
            if length == 4:
                param.append(Dynamixel.DXL_LOBYTE(Dynamixel.DXL_LOWORD(cmd)))
                param.append(Dynamixel.DXL_HIBYTE(Dynamixel.DXL_LOWORD(cmd)))
                param.append(Dynamixel.DXL_LOBYTE(Dynamixel.DXL_HIWORD(cmd)))
                param.append(Dynamixel.DXL_HIBYTE(Dynamixel.DXL_HIWORD(cmd)))
            elif length == 2:
                param.append(Dynamixel.DXL_LOBYTE(cmd))
                param.append(Dynamixel.DXL_HIBYTE(cmd))
            else:
                param.append(cmd)
            dxl_addparam_result = groupSyncWrite.addParam(id, param)
            if dxl_addparam_result != True:
                print("ID:%03d groupSyncWrite addparam failed" % id)
                return False
        dxl_comm_result = groupSyncWrite.txPacket()
        if dxl_comm_result != Dynamixel.COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            return False
        return True

    ### @brief Reads data from a group of motors synchronously
    ### @param groupSyncRead - groupSyncRead object
    ### @param ids - list of Dynamixel IDs to read from
    ### @param address - register number
    ### @param length - size of register in bytes
    ### @return states - list to store the requested data
    ### @return <bool> - true if data was successfully retrieved; false otherwise
    ### @details - DynamixelSDK uses 2's complement so we need to check to see if 'state' should be negative (hex numbers)
    def syncRead(self, groupSyncRead, ids, address, length):
        groupSyncRead.clearParam()
        for id in ids:
            dxl_addparam_result = groupSyncRead.addParam(id)
            if dxl_addparam_result != True:
                print("ID:%03d groupSyncRead addparam failed", id)
                return [], False

        dxl_comm_result = groupSyncRead.txRxPacket()
        if dxl_comm_result != Dynamixel.COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            return [], False

        states = []
        for id in ids:
            state = groupSyncRead.getData(id, address, length)
            if length == 2 and state > 0x7fff:
                state = state - 65536
            elif length == 4 and state > 0x7fffffff:
                state = state - 4294967296
            states.append(state)
        return states, True

    ### @brief Calibrates a dual-motor joint so that both motors show the same Present Position value
    ### @param master_id - in a dual-motor joint setup, this is the ID with the lower value
    ### @param slave_id - in a dual-motor joint setup, this is the ID with the higher value
    ### @details - note that the Drive mode must be set beforehand for both motors (specifically if CW or CCW is positive)
    def calibrateDualJoint(self, master_id, slave_id):
        self.itemWrite(slave_id, DXL_CONSTANTS.ADDR_HOMING_OFFSET, 0, DXL_CONSTANTS.LEN_HOMING_OFFSET)
        master_position, _ = self.itemRead(master_id, DXL_CONSTANTS.ADDR_PRESENT_POSITION,
                                           DXL_CONSTANTS.LEN_PRESENT_POSITION)
        slave_position, _ = self.itemRead(slave_id, DXL_CONSTANTS.ADDR_PRESENT_POSITION,
                                          DXL_CONSTANTS.LEN_PRESENT_POSITION)
        slave_drive_mode, _ = self.itemRead(slave_id, DXL_CONSTANTS.ADDR_DRIVE_MODE, DXL_CONSTANTS.LEN_DRIVE_MODE)
        slave_forward = (slave_drive_mode % 2 == 0)
        print("Master Position: %d" % master_position)
        print("Slave Position: %d" % slave_position)
        if slave_forward:
            homing_offset = master_position - slave_position
        else:
            homing_offset = slave_position - master_position
        print("Setting Slave Homing Offset Register to: %d" % homing_offset)
        self.itemWrite(slave_id, DXL_CONSTANTS.ADDR_HOMING_OFFSET, homing_offset, DXL_CONSTANTS.LEN_HOMING_OFFSET)

    ### @brief Ping the desired motors to verify their existence
    ### @param ids - vector of Dynamixel IDs to ping
    ### @return <bool> - true if all IDs were pinged successfully
    def ping(self, ids):
        for id in ids:
            model_num, dxl_comm_result, dxl_error = self.packetHandler.ping(self.portHandler, id)
            success = self.checkError(dxl_comm_result, dxl_error)
            if success:
                print("Pinged ID: %03d successfully! Model Number: %d" % (id, model_num))
            else:
                return False
        return True

    # ### @brief Convert a raw Position register value into radians
    # ### @param value - desired raw value to convert
    # ### @return <float> - value in radians
    # def convertValue2Radian(self, value):
    #     return (value - 2047.5) / 2047.5 * DXL_CONSTANTS.PI
    #
    # ### @brief Convert a value in radians to the raw Position register value
    # ### @param radian - desired value in radians to convert
    # ### @return <float> - raw register value
    # def convertRadian2Value(self, radian):
    #     return round(radian / DXL_CONSTANTS.PI * 2047.5 + 2047.5)
