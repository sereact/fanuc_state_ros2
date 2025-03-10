import random
import socket
import string
import struct
import time
from urllib import robotparser
from fanuc_state_ros2 import srtp_message

# UOP signal constants
# UI
opin_ui_imstp                       = 1
opin_ui_hold                        = 2
opin_ui_sfspd                       = 3
opin_ui_cstopi                      = 4
opin_ui_reset                       = 5
opin_ui_start                       = 6
opin_ui_home                        = 7
opin_ui_enbl                        = 8
opin_ui_pns1                        = 9
opin_ui_pns2                        = 10
opin_ui_pns3                        = 11
opin_ui_pns4                        = 12
opin_ui_pns5                        = 13
opin_ui_pns6                        = 14
opin_ui_pns7                        = 15
opin_ui_pns8                        = 16
opin_ui_strobe                      = 17
opin_ui_prod_start                  = 18
# UO
opout_uo_cmdenbl                    = 1
opout_uo_sysrdy                     = 2
opout_uo_progrun                    = 3
opout_uo_paused                     = 4
opout_uo_held                       = 5
opout_uo_fault                      = 6
opout_uo_atperch                    = 7
opout_uo_tpenbl                     = 8
opout_uo_batalm                     = 9
opout_uo_busy                       = 10
opout_uo_pns1                       = 11
opout_uo_pns2                       = 12
opout_uo_pns3                       = 13
opout_uo_pns4                       = 14
opout_uo_pns5                       = 15
opout_uo_pns6                       = 16
opout_uo_pns7                       = 17
opout_uo_pns8                       = 18
opout_uo_snack                      = 19

from threading import Lock

class RobotComm:
    def __init__(self, ip, port):
        self.ip = ip
        self.port = port
        self._sock = self.open_socket(ip, port)
        self.lock = Lock()

    def __del__(self):
        self._sock.close()
    

    @property
    def sock(self):
        with self.lock:
            return self._sock
    

    def open_socket(self, robot_ip, snpx_port):
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.connect((robot_ip, snpx_port))
        s.send(srtp_message.INIT_MSG)
        init_resp = s.recv(1024)
        if init_resp[0] != 1:
            raise Exception("Init Response: ", init_resp[0])
        init_comm = self.read_mem(0, "R", s)
        if init_comm == 256:
            return s
        else:
            raise Exception("Communication Fault: ", init_comm)


    def read_mem(self, addr, reg, s, size=1):
        msg = srtp_message.BASE_MSG.copy()
        msg[42] = srtp_message.SERVICE_REQUEST_CODE["READ_SYS_MEMORY"]
        msg[43] = srtp_message.MEMORY_TYPE_CODE[reg]
        if addr > 0:    # Init Check, if not the init add data.
            msg[44] = int(int((addr - 1) / 8) & 255).to_bytes(1, byteorder='big')
            if (reg == "AI" or reg == "R") or size > 1:
                msg[44] = int(addr - 1 & 255).to_bytes(1, byteorder='big')
            msg[45] = int(addr - 1 >> 8).to_bytes(1, byteorder='big')
        msg[46] = int(size).to_bytes(1, byteorder='big')
        out_bytes = b''.join(msg)

        s.send(out_bytes)
        r = s.recv(1024)

        if addr == 0:  # Init Comm Check
            return struct.unpack('H', bytearray(r[47:49]))[0]
        # print(decode_packet(out_bytes))
        return r


    # Maybe seperate this into num regs, strings, I/O
    def write_mem(self, addr, reg, var, s):
        msg = srtp_message.BASE_MSG.copy()
        if type(var) == int:  # Var length might always be 2
            fill = False
            var_length = int(len(format(int(var & 255), '02X') + format(int(var >> 8), '02X')) / 2)  # Could be simplified
        elif type(var) == float:    # Var length might always be 4
            fill = False
            var_length = int((len(hex(struct.unpack('<I', struct.pack('<f', 123.456))[0]))-2) / 2)
        elif type(var) == bool:
            fill = False
            var_length = 1
        elif len(var) % 2:
            fill = True
            var_length = len(var)+1
        else:
            fill = False
            var_length = len(var)
        msg[4] = var_length.to_bytes(1, byteorder='big')
        msg[9] = b'\x02'    # Base Message set to Read, x02 is for Write
        msg[17] = b'\x02'   # Base Message set to Read, x02 is for Write
        msg[30] = b'\x09'   # Seq Number - 0x09 for Writing
        msg[31] = b'\x80'   # Message Type - Kepware shows \x80 for writing
        msg[42] = var_length.to_bytes(1, byteorder='big')
        msg[48] = b'\x01'
        msg[49] = b'\x01'
        msg[50] = srtp_message.SERVICE_REQUEST_CODE["WRITE_SYS_MEMORY"]
        msg[51] = srtp_message.MEMORY_TYPE_CODE[reg]
        msg[52] = int(addr - 1 & 255).to_bytes(1, byteorder='big')
        msg[53] = int(addr - 1 >> 8).to_bytes(1, byteorder='big')
        msg[54] = (int(var_length / 2)).to_bytes(1, byteorder='big')
        if type(var) == str:
            for data in var:
                msg.append(data.encode('utf8'))
        elif type(var) == float:
            msg.append(bytearray(struct.pack("f", var)))
        else:
            msg.append(int(var & 255).to_bytes(1, byteorder='big'))
            msg.append(int(var >> 8).to_bytes(1, byteorder='big'))
        if type(var) == bool:
            msg[54] = b'\x01'   # Write Length always 1 for bits
            idx = ((addr - 1) % 8)
            if var:
                result = (idx * "0" + "1" + ((8 - idx) * "0"))[::-1]
            else:
                result = "0"
            msg[56] = (int(result, 2).to_bytes(1, byteorder='big'))
            msg.pop()
        if fill:
            msg.append(b'\x00')

        out_bytes = b''.join(msg)
        # print(decode_packet(out_bytes))
        s.send(out_bytes)
        return s.recv(1024)


    # For Decoding Registers (GOs and Numeric Registers)
    def decode_register(self, r):
        return struct.unpack('H', bytearray(r[44:46]))[0]


    # Decoding Floats (SNPX Multiply set to 0 not 1)
    def decode_float(self, r):
        return struct.unpack('!f', bytes.fromhex(decode_packet(r[44:48][::-1])))[0]


    # For Decoding Words (Program Line: PRG[1] in SNPX, etc.)
    def decode_word(self, r):
        return struct.unpack('H', bytearray(r[56:58]))[0]


    # For decoding DOs and DIs
    def decode_bit(self, r, addr):
        return format(int(struct.unpack('H', bytearray(r[44:46]))[0]), '08b')[::-1][(addr - 1) % 8]


    def decode_string(self, r):
        return r[56:].decode().rstrip("\x00")


    def decode_packet(self, msg):
        iter_len = sum(1 for _ in enumerate(msg))
        out = ""
        for idx, i in enumerate(msg):
            if iter_len - idx <= iter_len:
                out += " {:02x}".format(i)
        return out

    def get_di(self, di_nr):
        return self.decode_bit(self.read_mem(di_nr, "Q", self.sock), di_nr) == '1'
        
    def set_do(self, do_nr, value):
        self.write_mem(do_nr, "DO",  value, self.sock)


    # 
    def ClearUopInput(self):
        """
        ClearUopInput sets I/O to clean default state to make the system operable
        """
        self.set_do(opin_ui_imstp, True)
        self.set_do(opin_ui_hold, True)
        self.set_do(opin_ui_sfspd, True)
        self.set_do(opin_ui_cstopi, False)
        self.set_do(opin_ui_reset, False)
        # Last used program could start if start bit is on, so we don't change it
        self.set_do(opin_ui_home, False)
        self.set_do(opin_ui_enbl, True)

        # For same reason don't clear PROD_START UI

        for UONr in range(9, opin_ui_prod_start):
            self.set_do(UONr, False)


    def IsFaultReset(self) -> bool:
        """
        IsFaultReset returns if fault is reset
        """
        if (self.get_di(opout_uo_tpenbl)):
            return (self.get_di(opout_uo_fault) == False)
        else:
            return (self.get_di(opout_uo_fault) == False) and (self.get_di(opout_uo_cmdenbl) == True)


    def FaultReset(self) -> bool:
        """
        FaultReset performs fault reset over UOP interface
        """
        if (not self.IsFaultReset()):
            # Pulse reset
            self.set_do(opin_ui_reset, True)
            time.sleep (0.100)
            self.set_do(opin_ui_reset, False)
            
            timeout = 0
            while not self.IsFaultReset() and timeout < 1000:
                time.sleep (0.050)
                timeout = timeout + 50

        if (self.get_di(opout_uo_fault) == True):
            print ('Fault not reset')

            if (self.get_di(opout_uo_tpenbl) == True):
                print ('TP enabled')

            return (False)

            return (False)

        if (self.get_di(opout_uo_tpenbl) == False) and (self.get_di(opout_uo_cmdenbl) == False):
            print ('CMDENBL False')
            return (False)

        return (True)


    def IsSystemReady(self) -> bool:
        """
        IsSystemReady returns if system is ready for commands dependent on
        operation mode
        """
        if (self.get_di(opout_uo_tpenbl) == True):
            return (self.get_di(opout_uo_fault) == False) and (self.get_di(opout_uo_sysrdy) == True)
        else:
            return (self.get_di(opout_uo_cmdenbl) == True) and (self.get_di(opout_uo_sysrdy) == True)



    def ReadyForPrgRun(self) -> bool:
        """
        ReadyForPrgRun checks if UOP interface is ready for program run.
        """
        # Program running
        if self.get_di(opout_uo_progrun):
            print ('Program running')
            return (False)

        if (self.FaultReset() == False):
            print ('Fault not reset')
            return (False)

        timeout = 0
        while not self.IsSystemReady() and timeout < 1000:
            time.sleep (0.050)
            timeout = timeout + 50

        return (timeout < 1000)


    def PrgStartUOP(self, PNSNr) -> bool:
        """
        PrgStartUOP starts program over UOP interface
        """
        # Program actually starts on falling edge
        self.set_do(opin_ui_prod_start, True)

        # Set PNS Nr. bit by bit
        for BitNr in range (8):
            if (PNSNr % 2) > 0:
                self.set_do(opin_ui_pns1 + BitNr, True)

            # Right shift
            PNSNr = PNSNr // 2

        time.sleep (0.020)
        self.set_do(opin_ui_strobe, True)
        time.sleep (0.100)
        self.set_do(opin_ui_strobe, False)
        timeout = 0
        while not self.get_di(opout_uo_snack) and timeout < 600:
            time.sleep (0.016)
            timeout = timeout + 16

        if (timeout > 600):
            print ('No SNACK')
            return (False)

        self.set_do(opin_ui_prod_start, False)

        timeout = 0
        # Until Prg running
        while not self.get_di(opout_uo_progrun) and timeout < 600:
            time.sleep (0.050)
            timeout = timeout + 50

        return (timeout < 600)



    def IsPrgRunning(self) -> bool:
        """
        IsPrgRunning queries over UOP interface if a program is running
        """
        if self.get_di(opout_uo_progrun):
            return (True)
        else:
            return (False)
    

    def PrgPause(self) -> bool:
        """
        PrgPause pauses program over UOP interface
        """
        self.ClearUopInput()
                
        self.set_do(opin_ui_enbl, False)

        timeout = 0
        # Until Prg paused
        while not self.get_di(opout_uo_paused) and timeout < 600:
            time.sleep (0.050)
            timeout = timeout + 50   

        return (timeout < 600)


    def PrgContinue(self) -> bool:
        """
        PrgContinue continues execution of a paused program over UOP interface
        """
        self.set_do(opin_ui_imstp, True)
        self.set_do(opin_ui_hold, True)
        self.set_do(opin_ui_sfspd, True)
        self.set_do(opin_ui_cstopi, False)
        self.set_do(opin_ui_reset, False)
        self.set_do(opin_ui_start, False)
        self.set_do(opin_ui_home, False)
        self.set_do(opin_ui_enbl, True)

        if (not self.ReadyForPrgRun()):
            return (False)

        self.set_do(opin_ui_start, True)      
        time.sleep (0.100)
        self.set_do(opin_ui_start, False)   

        timeout = 0
        # Until Prg running
        while not self.get_di(opout_uo_progrun) and timeout < 1000:
            time.sleep (0.050)
            timeout = timeout + 50

        return (timeout < 1000)



    def PrgAbort(self) -> bool:
        """
        PrgAbort aborts all programs over UOP interface
        """
        self.ClearUopInput()
        self.set_do(opin_ui_cstopi, True) 

        timeout = 0
        while True:
            time.sleep (0.050)
            timeout = timeout + 50
            # Until Prg running and paused are False
            prg_aborted = (self.get_di(opout_uo_progrun) == False) and (self.get_di(opout_uo_paused) == False)
            if prg_aborted or (timeout > 1000):
                break

        self.set_do(opin_ui_cstopi, False) 

        return (prg_aborted)


    def RobotIMSTP(self) -> bool:
        """
        RobotIMSTP: Immediate Stop of robot and pause of programs over UOP interface
        """
        self.set_do(opin_ui_imstp, False) 
        self.set_do(opin_ui_enbl, False) 

        timeout = 0
        # Until Prg running is False
        while self.get_di(opout_uo_progrun) and timeout < 600:
            time.sleep (0.050)
            timeout = timeout + 50

        return (timeout < 600)

# In SYSTEM -> Config, enable UOP signals,
# Set CSTOPI for abort to TRUE,
# Set remote / local setup to remote
# In SETUP -> Prog Select, set Program select mode to PNS and Production start method to UOP
# Map DI[1-20] to rack 0, slot 0, start 1
# Map DO[1-18] to rack 0, slot 0, start 21
# Map UI[1-18] to rack 0, slot 0, start 21
# Map UO[1-20] to rack 0, slot 0, start 1

if __name__ == '__main__':
    port = 18245
    ip = "192.168.1.100"
    comm = RobotComm(ip, port)
    comm.ClearUopInput()
    comm.PrgAbort()
    comm.FaultReset()
    print(comm.get_di(opout_uo_fault))