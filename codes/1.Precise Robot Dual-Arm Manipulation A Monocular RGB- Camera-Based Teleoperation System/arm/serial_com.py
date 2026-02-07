import json
import serial
import pdb

# 初始化变量
cmd_loc = [0]*6
cmd_spd = [0]*6
cmd_hand_mode = 0
feedback_loc = [0]*6
feedback_spd = [0]*6
feedback_hand_mode = 0

# 初始化串口


class Serial_com:
    def __init__(self):
        self.sp = serial.Serial('/dev/ttyUSB1', 115200, timeout=1, parity=serial.PARITY_NONE, bytesize=serial.EIGHTBITS)
        self.cmd_loc = [0]*6
        self.cmd_spd = [0]*6
        self.cmd_hand_mode = 0
        self.feedback_loc = [0]*6
        self.feedback_spd = [0]*6
        self.feedback_hand_mode = 0
        self.open()

    def open(self):
        try:
            self.sp.open()
            return True
        except serial.SerialException as e:
            return False

    def json_out(self):
        data = {
            "host": self.cmd_loc + self.cmd_spd + [self.cmd_hand_mode]
        }
        # 检查输出缓冲区是否为空，如果不为空，等待直到空
        while self.sp.out_waiting:
            pass
        self.sp.write(json.dumps(data).encode())
        # print(data)

    # def json_input(self):
    #     # 检查输入缓冲区是否有数据，如果没有，等待直到有数据
    #     while not self.sp.in_waiting:
    #         pass
    #     data = ""
    #     while not data.endswith("\n"):
    #         data += self.sp.read_all().decode()
    #     if data:
    #         try:
    #             data = json.loads(data)
    #             master_data = data.get("master", None)
    #             if master_data is not None:
    #                 self.feedback_loc = master_data[:6]
    #                 self.feedback_spd = master_data[6:12]
    #                 self.feedback_hand_mode = master_data[12]
    #             else:
    #                 print("Invalid JSON data received: 'master' key not found.")
    #         except json.JSONDecodeError:
    #             print("Invalid JSON data received.")

    # 鲁棒版本
    def json_input(self):
        # Check if there is any data in the input buffer, if not, wait until there is data
        while not self.sp.in_waiting:
            pass

        buffer = ""
        while True:
            try:
                # Read all available data from the serial port
                buffer += self.sp.read(self.sp.in_waiting).decode('utf-8', errors='ignore')

                # Attempt to find a complete JSON message ending with a newline character
                if '\n' in buffer:
                    # Split the buffer at the first newline to extract the complete JSON message
                    complete_msg, buffer = buffer.split('\n', 1)
                    data = json.loads(complete_msg)  # Parse the complete JSON message
                    master_data = data.get("master", None)
                    if master_data is not None and len(master_data) == 13:  # TOO 这里的可以改为==13
                        self.feedback_loc = master_data[:6]
                        self.feedback_spd = master_data[6:12]
                        self.feedback_hand_mode = master_data[12]
                        return  # Exit the method after successful parsing
                    # else:
                    #     print("Invalid JSON structure or incomplete 'master' data.")
            except json.JSONDecodeError as e:
                pass
                # print(f"JSON decoding failed: {e}")
                # Clear the buffer since the data is not recoverable
                buffer = ""
            except UnicodeDecodeError as e:
                pass
                # print(f"Unicode decoding failed: {e}")
                # Skip the problematic part and continue with the next part of the buffer
            except Exception as e:
                # print(f"An unexpected error occurred: {e}")
                break  # Exit the loop if an unexpected error occurs

    def set_cmd_loc(self, cmd_loc):
        self.cmd_loc = cmd_loc

    def set_cmd_spd(self, cmd_spd):
        self.cmd_spd = cmd_spd

    def set_cmd_hand_mode(self, cmd_hand_mode):
        self.cmd_hand_mode = cmd_hand_mode

    def get_feedback_loc(self):
        return self.feedback_loc

    def get_feedback_spd(self):
        return self.feedback_spd
