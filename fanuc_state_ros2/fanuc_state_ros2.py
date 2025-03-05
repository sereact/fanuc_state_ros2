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

import os
import json
logger = SereactLogger(__name__)

CONFIG_FILE_PATH = os.environ.get("CONFIG_FILE_PATH", "assets/config_files/config.json")
config = {
    "UO":
    {
        "CMD_ENABLED": "/io/main/main/in_5_A/state",
        "FAULT": "/io/main/main/in_5_A/state",
        "BATT_ALARM": "/io/main/main/in_5_A/state",
        "BUSY": "/io/main/main/in_5_A/state"
    },
    "UI":
    {
        "IMSTP": "/io/main/main/out_5_A/set",
        "HOLD": "/io/main/main/out_5_A/set",
        "SFSPD": "/io/main/main/out_5_A/set",
        "CYCLE_STOP": "/io/main/main/out_5_A/set",
        "FAULT_RESET": "/io/main/main/out_5_A/set",
        "START": "/io/main/main/out_5_A/set",
        "ENABLE": "/io/main/main/out_5_A/set",
        "PNS1": "/io/main/main/out_5_A/set",
        "PNS2": "/io/main/main/out_5_A/set",
        "PNS3": "/io/main/main/out_5_A/set",
        "PNS_STROBE": "/io/main/main/out_5_A/set",
        "PROD_START": "/io/main/main/out_5_A/set"
    }

}


ROBOT_STATUS_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1
)

class FanucStateRos2(Node):

    def __init__(self):
        super().__init__('fanuc_state_ros2')
        self.config = json.load(open(CONFIG_FILE_PATH))


        assert "UI" in self.config, "UI key not found in config file"
        assert "UO" in self.config, "UO key not found in config file"
        
        self.name = "fanuc_state_ros2"
        self.current_karton_id = ""
        self.states = {}
        self.bit_publisher = {}

        
        
        self.robot_status = RobotStatus()
        
    

        for key, topic in self.config["UI"].items():
            self.bit_publisher[key] = self.create_publisher(Bool, topic, 10)

        self.subscriptions_callback_group = ReentrantCallbackGroup()

        for key, items in self.config["UO"].items():
            self.states[key] = False
            self.create_subscription(Bool, items, lambda msg, key=key: self.state_callback(msg, key=key), 10, callback_group=self.subscriptions_callback_group)

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
        # self.hold_publishing_timer = self.create_timer(5, self.hold_publishing_callback, callback_group=self.state_timer_callback_group)

    def send_false_all(self):
        self.bit_publisher["HOLD"].publish(self.false_msg)
        self.bit_publisher["ENABLE"].publish(self.false_msg)
        self.bit_publisher["PNS_STROBE"].publish(self.false_msg)
        self.bit_publisher["PNS1"].publish(self.false_msg)
        self.bit_publisher["PROD_START"].publish(self.false_msg)
        time.sleep(0.1)


    def start_program_callback(self, request, response):
        try:
            if self.states["FAULT"]:
                raise Exception("Robot is in fault state")

            if self.states["PROGON"] or self.states["PAUSED"]:
                self.stop_program()
            
            # self.send_false_all()
            self.bit_publisher["HOLD"].publish(self.true_msg)
            self.bit_publisher["IMSTP"].publish(self.true_msg)
            self.bit_publisher["SFSPD"].publish(self.true_msg)
            time.sleep(0.1)

            self.bit_publisher["ENABLE"].publish(self.true_msg)
            time.sleep(0.1)
            self.bit_publisher["PNS1"].publish(self.true_msg)
            self.bit_publisher["PROD_START"].publish(self.true_msg)
            time.sleep(0.1)

            self.bit_publisher["PNS_STROBE"].publish(self.true_msg)
            time.sleep(0.1)
            self.bit_publisher["PNS_STROBE"].publish(self.false_msg)
            time.sleep(0.1)
            response.response = True

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

    def hold_publishing_callback(self):
        self.bit_publisher["HOLD"].publish(self.true_msg)
        self.bit_publisher["IMSTP"].publish(self.true_msg)
        self.bit_publisher["SFSPD"].publish(self.true_msg)

    def stop_program(self):
        self.bit_publisher["HOLD"].publish(self.false_msg)
        self.bit_publisher["CYCLE_STOP"].publish(self.true_msg)
        time.sleep(0.1)
        self.bit_publisher["CYCLE_STOP"].publish(self.false_msg)

    def stop_program_callback(self, request, response):
        self.stop_program()
        return response

    def reset_fault_callback(self, request, response):
        self.bit_publisher["FAULT_RESET"].publish(self.true_msg)
        time.sleep(0.5)
        self.bit_publisher["FAULT_RESET"].publish(self.false_msg)
        response.response = True
        return response

    def state_timer_callback(self):
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
