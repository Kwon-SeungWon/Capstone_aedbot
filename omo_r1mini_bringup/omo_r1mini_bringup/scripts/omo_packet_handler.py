#!/usr/bin/env python3

import serial
import io
from time import sleep

class ReadLine:
   def __init__(self, s):
      self.buf = bytearray()
      self.s = s

   def readline(self):
      i = self.buf.find(b"\n")
      if i >= 0:
         r = self.buf[:i+1]
         self.buf = self.buf[i+1:]
         return r
      while True:
         i = max(1, min(2048, self.s.in_waiting))
         data = self.s.read(i)
         i = data.find(b"\n")
         if i >= 0:
            r = self.buf + data[:i+1]
            self.buf[0:] = data[i+1:]
            return r
         else:
            self.buf.extend(data)

class PacketHandler:
   def __init__(self, _port_name, _baud_rate):
      #_port_name = "/dev/ttyUSB0"
      _port_name = "/dev/ttyMotor"
      _baud_rate = 115200
      self.port_name = _port_name
      self.baud_rate = _baud_rate
      self._ser = serial.Serial(self.port_name, self.baud_rate)
      self._ser_io = io.TextIOWrapper(io.BufferedRWPair(self._ser, self._ser, 1), 
                                       newline = '\r', 
                                       line_buffering = True)
      self._rl = ReadLine(self._ser)
      print("Serial_port: ", _port_name)
      print("Serial baud rate: ", _baud_rate)
      self.write_periodic_query_enable(0)
      self._ser.flushInput()
      self._ser.reset_input_buffer()
      self._ser.reset_output_buffer()
      self.incomming_info = ['ODO', 'VW', 'POSE', 'ACCL', 'GYRO']
      self._vel = [0.0, 0.0]
      self._enc = [0.0, 0.0]
      self._enc_off = [0.0,0.0]
      self._off = False
      self._wodom = [0.0, 0.0]
      self._rpm = [0.0, 0.0]
      self._wvel = [0.0, 0.0]
      self._gyro = [0.0, 0.0, 0.0]
      self._imu = [0.0, 0.0, 0.0]
      self._battery = [0.0, 0.0, 0.0]

   def set_periodic_info(self, param1):
      for idx, each in enumerate(self.incomming_info):
         #print("$cREGI," + str(idx) + "," + each)
         self.write_port("$cREGI," + str(idx) + "," + each)

      self.write_port("$cPERI," + str(param1))
      sleep(0.01)
      self.write_periodic_query_enable(1)

   def get_port_state(self):
      return self._ser.isOpen()
      
   def read_port(self):
      return self._rl.readline()

   def close_port(self):
      print("Port close")
      self._ser.close()

   def read_packet(self):
      if self.get_port_state() == True:

         #whole_packet = (self._ser.readline().split(b'\r')[0]).decode("utf-8").strip()
         whole_packet = self.read_port()
         #print(whole_packet)   
         if whole_packet:
            #print(whole_packet)
            #packet = whole_packet.split(',')
            packet = whole_packet.split(b',')
            try:
               header = packet[0].split(b'#')[1]
               if header.startswith(b'QVW'):
                  self._vel = [float(packet[1]), float(packet[2])]
               elif header.startswith(b'QENCOD'):
                  self._enc = [int(packet[1]), int(packet[2])]
               elif header.startswith(b'QODO'):
                  self._wodom = [float(packet[1]), float(packet[2])]
               elif header.startswith(b'QRPM'):
                  self._rpm = [int(packet[1]), int(packet[2])]
               elif header.startswith(b'QDIFFV'):
                  self._wvel = [int(packet[1]), int(packet[2])]
               elif header.startswith(b'GYRO'):
                  self._gyro = [float(packet[1]), float(packet[2]), float(packet[3])]
               elif header.startswith(b'POSE'):
                  self._imu = [float(packet[1]), float(packet[2]), float(packet[3])]
               elif header.startswith(b'BAT'):
                  self._battery = [float(packet[1]), float(packet[2]), float(packet[3])]
            except:
               pass
   
   def update_battery_state(self):
      self.write_port("$qBAT")
      sleep(0.01)

   def get_base_velocity(self):
      return self._vel

   def get_wheel_encoder(self):
      return self._enc

   def get_wheel_odom(self):
      return self._wodom

   def get_wheel_rpm(self):
      return self._rpm

   def get_wheel_velocity(self):
      return self._wvel

   def get_battery_status(self):
      return self._battery

   def write_periodic_query_enable(self, param):
      self.write_port("$cPEEN," + str(param))
      sleep(0.05)

   def write_odometry_reset(self):
      self.write_port("$cODO,0")
      sleep(0.05)

   def write_base_velocity(self, lin_vel, ang_vel):
      # lin_vel : mm/s, ang_vel : mrad/s
      self.write_port('$CVW,{:.0f},{:.0f}'.format(lin_vel, ang_vel))

   def write_wheel_velocity(self, wheel_l_lin_vel, wheel_r_lin_vel):
      self.write_port('$CDIFFV,{:.0f},{:.0f}'.format(wheel_l_lin_vel, wheel_r_lin_vel))

   def write_port(self, buffer):
      if self.get_port_state() == True:
         self._ser.write((buffer + "\r\n").encode())
         
         # a = bytes(buffer + "\r\n", 'utf-8')
         # print(a)
         # self._ser.write(a)

         #self._ser.write(buffer + "\r\n")

###################################################
# class PacketWriteHandler:
#    _ph = None

#    def __init__(self, ph):
#       self._ph = ph
#       msg = "0"
#       self.write_packet(msg.encode())

#    def __del__(self):
#       self._ph = None

#    def write_register(self, param1, param2):
#       self.write_packet("$SREGI," + str(param1) + ',' + param2)
#       sleep(0.05)

#    def write_periodic_query_value(self, param):
#       self.write_packet("$SPERI," + str(param))
#       sleep(0.05)

#    def write_periodic_query_enable(self, param):
#       self.write_packet("$SPEEN," + str(param))
#       sleep(0.05)

#    def write_init_odometry(self):
#       self.write_packet("$SODO")
#       sleep(0.05)

#    def write_wheel_velocity(self, wheel_l_lin_vel, wheel_r_lin_vel):
#       self.write_packet('$CDIFFV,{:.0f},{:.0f}'.format(wheel_l_lin_vel, wheel_r_lin_vel))

#    def write_base_velocity(self, lin_vel, ang_vel):
#       # lin_vel : mm/s, ang_vel : mrad/s
#       self.write_packet('$CVW,{:.0f},{:.0f}'.format(lin_vel, ang_vel))

#    def write_packet(self, packet):
#       if self._ph.get_port_state() == True:
#          self._ph.write_port(packet)

#    def stop_peen(self):
#       self.write_packet("$SPEEN, 0")
#       sleep(0.05)

#    def stop_callback(self):
#       self.write_packet("$SCBEN, 0")
#       sleep(0.05)
      
# class ReadLine:
#    def __init__(self, s):
#       self.buf = bytearray()
#       self.s = s

#    def readline(self):
#       i = self.buf.find(b"\n")
#       if i >= 0:
#          r = self.buf[:i+1]
#          self.buf = self.buf[i+1:]
#          return r
#       while True:
#          i = max(1, min(2048, self.s.in_waiting))
#          data = self.s.read(i)
#          i = data.find(b"\n")
#          if i >= 0:
#             r = self.buf + data[:i+1]
#             self.buf[0:] = data[i+1:]
#             return r
#          else:
#             self.buf.extend(data)

# class PacketReadHandler:
#    _ph = None

#    # velocity
#    _vel = None
#    # encoder
#    _enc = None
#    # odometry
#    _odom = None
#    # rpm
#    _rpm = None
#    # wheel velocity
#    _wvel = None

#    def __init__(self, ph):
#       self._ph = ph

#       self._vel = [0.0, 0.0]
#       self._enc = [0.0, 0.0]
#       self._wodom = [0.0, 0.0]
#       self._rpm = [0.0, 0.0]
#       self._wvel = [0.0, 0.0]

#    def __del__(self):
#       self._ph = None

#       self._vel = None
#       self._enc = None
#       self._wodom = None
#       self._rpm = None
#       self._wvel = None

#    def get_base_velocity(self):
#       return self._vel
   
#    def get_wheel_encoder(self):
#       return self._enc

#    def get_wheel_odom(self):
#       return self._wodom

#    def get_wheel_rpm(self):
#       return self._rpm
   
#    def get_wheel_velocity(self):
#       return self._wvel

#    def read_packet(self):
#       if self._ph.get_port_state() == True:
#          whole_packet = self._ph.read_port()
#          if whole_packet:
#             packet = whole_packet.split(",")
#             try:
#                header = packet[0].split("#")[1]
               
#                if header.startswith('QVW'):
#                   self._vel = [int(packet[1]), int(packet[2])]
#                elif header.startswith('QENCOD'):
#                   self._enc = [int(packet[1]), int(packet[2])]
#                elif header.startswith('QODO'):
#                   self._wodom = [int(packet[1]), int(packet[2])]
#                elif header.startswith('QRPM'):
#                   self._rpm = [int(packet[1]), int(packet[2])]
#                elif header.startswith('QDIFFV'):
#                   self._wvel = [int(packet[1]), int(packet[2])]
#             except:
#                pass
# class PortHandler():
#    _port_name = None
#    _baud_rate = None
#    _ser = None
#    _ser_io = None
#    _rl = None

#    def __init__(self, port_name, baud_rate):
#       self._port_name = port_name
#       self._baud_rate = baud_rate

#       self.set_port_handler(port_name, baud_rate)

#    def __del__(self):
#       self._ser.close()

#    def set_port_handler(self, port_name, baud_rate):
#       self._ser = serial.Serial(port_name, baud_rate)
#       self._ser_io = io.TextIOWrapper(io.BufferedRWPair(self._ser, self._ser, 1), newline = '\r', line_buffering = True)
#       self._rl = ReadLine(self._ser)
#    def get_port_handler(self):
#       return self._ser

#    def get_port_name(self):
#       return self._port_name

#    def get_port_baud_rate(self):
#       return self._baud_rate

#    def get_port_state(self):
#       return self._ser.isOpen()

#    def write_port(self, buffer):
#       if(type(buffer) is str):
#          buffer.encode()
#       print(type("\r\n"))
#       self._ser.write(buffer + "\r\n")

#    def read_port(self):
#       return self._rl.readline()
################################################################