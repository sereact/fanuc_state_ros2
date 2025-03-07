from enum import Enum
import re
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Bool
from sereact_custom_messaging.srv import BoolSrv
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from fanuc_state_ros2.logger import SereactLogger
from industrial_msgs.msg import RobotStatus, TriState, RobotMode
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from fanuc_state_ros2 import robot_comm
from fanuc_state_ros2.robot_comm import RobotComm
import os
import json
logger = SereactLogger(__name__)

CONFIG_FILE_PATH = os.environ.get("CONFIG_FILE_PATH", "assets/config/config.json")

STATE_CONFIG = {
    "CMD_ENABLED": robot_comm.opout_uo_cmdenbl,
    "FAULT": robot_comm.opout_uo_fault,
    "BATT_ALARM": robot_comm.opout_uo_batalm,
    "BUSY": robot_comm.opout_uo_busy,
    "PAUSED": robot_comm.opout_uo_paused,
    "PROGON": robot_comm.opout_uo_progrun
}


ROBOT_STATUS_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1
)

class FanucStateRos2(Node):

    def __init__(self):
        super().__init__('fanuc_state_ros2')
        self.state_config = STATE_CONFIG
        self.robot_config = json.load(open(CONFIG_FILE_PATH))
        assert "ROBOT_IP" in self.robot_config, "ROBOT_IP key not found in config file"
        assert "PORT" in self.robot_config, "PORT key not found in config file"
        self.robot_ip_comm = RobotComm(self.robot_config["ROBOT_IP"], self.robot_config["PORT"])

        self.name = "fanuc_state_ros2"
        self.current_karton_id = ""
        self.states = {}
        self.bit_publisher = {}

        
        
        self.robot_status = RobotStatus()

        for key, items in self.state_config.items():
            self.states[key] = False

        self.true_msg = Bool()
        self.true_msg.data = True
        self.false_msg = Bool()
        self.false_msg.data = False
        self.services_callback_group = MutuallyExclusiveCallbackGroup()
        self.create_service(BoolSrv, self.name+"/start_program", self.start_program_callback, callback_group=self.services_callback_group)
        self.create_service(BoolSrv ,self.name+"/stop_program", self.stop_program_callback, callback_group=self.services_callback_group)
        self.create_service(BoolSrv ,self.name+"/reset_fault", self.reset_fault_callback, callback_group=self.services_callback_group)
        
        self.robot_status = RobotStatus()

        self.robot_status_publisher = self.create_publisher(RobotStatus, "/robot_status", ROBOT_STATUS_QOS)
        self.state_timer_callback_group = MutuallyExclusiveCallbackGroup()
        self.state_timer = self.create_timer(1/10, self.state_timer_callback, callback_group=self.state_timer_callback_group)
        self.robot_ip_comm.ClearUopInput()
        logger.info("FanucStateRos2 started")

    def send_false_all(self):
        self.robot_ip_comm.ClearUopInput()


    def start_program_callback(self, request, response):
        try:
            if self.states["FAULT"]:
                raise Exception("Robot is in fault state")

            if self.states["PROGON"] or self.states["PAUSED"]:
                logger.warn("Program is still running. Aborting the current program")
                self.robot_ip_comm.PrgAbort()

            if self.robot_ip_comm.ReadyForPrgRun():
                self.robot_ip_comm.PrgStartUOP(1)
                response.response = True
            else:
                raise Exception("Robot is not ready to run")

        except Exception as e:
            response.response = False
            response.exception_msg = str(e)
            response.exception_type = "Exception"
            response.raise_exception = True
            logger.error(e)
        return response

    def state_callback(self, msg, key):
        self.states[key] = msg.data
        
    # Service ca
    # Timer callback    
    def stop_program(self):
        self.robot_ip_comm.PrgAbort()

    def stop_program_callback(self, request, response):
        self.stop_program()
        response.response = True
        return response

    def reset_fault_callback(self, request, response):
        self.robot_ip_comm.FaultReset()
        response.response = True
        return response

    def state_timer_callback(self):
        for state_key, bit in STATE_CONFIG.items():
            self.states[state_key] = self.robot_ip_comm.get_di(bit)

        if self.states["FAULT"]:
            self.robot_status.in_error.val = TriState.TRUE
            self.robot_status.e_stopped.val = TriState.TRUE
        else:
            self.robot_status.e_stopped.val = TriState.FALSE
            self.robot_status.in_error.val = TriState.FALSE
            
        self.robot_status_publisher.publish(self.robot_status)
        
def main():
    rclpy.init()
    executor = MultiThreadedExecutor(4)
    device_node = FanucStateRos2()
    executor.add_node(device_node)
    executor.spin()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
