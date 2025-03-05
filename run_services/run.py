import os
import multiprocessing

from fanuc_state_ros2.fanuc_state_ros2 import main as fanuc_main
import time

class Manager:
    def __init__(self):
        self.processes = {}

    def add_process(self, process_name, process_function, args=[]):
        self.processes[process_name] = multiprocessing.Process(target=process_function, args=args, daemon=True)
        self.processes[process_name].start()
        
        
    def kill_process(self, process_name):
        self.processes[process_name].terminate()
        self.processes[process_name].join()
        
    def kill_proceses(self):
        for process_name in self.processes:
            self.kill_process(process_name)

    def main(self):
        self.add_process("fanuc_state", fanuc_main)

        
    def close(self):
        self.kill_proceses()

if __name__ == "__main__":
    manager = Manager()
    manager.main()
    try:
        while True:
            time.sleep(10)
    except KeyboardInterrupt:
        print("Exiting...")
    manager.close()

