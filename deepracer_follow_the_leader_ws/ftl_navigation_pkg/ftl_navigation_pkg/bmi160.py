from smbus2 import SMBus
import sys, getopt 
from time import sleep
import os

bus=SMBus(2)

BMI160_DEVICE_ADDRESS = 0x68

BMI160_REGA_USR_CHIP_ID      = 0x00
BMI160_REGA_USR_ACC_CONF_ADDR     = 0x40 
BMI160_REGA_USR_ACC_RANGE_ADDR    = 0x41
BMI160_REGA_USR_GYR_CONF_ADDR     = 0x42
BMI160_REGA_USR_GYR_RANGE_ADDR    = 0x43

BMI160_REGA_CMD_CMD_ADDR          =   0x7e
BMI160_REGA_CMD_EXT_MODE_ADDR     =   0x7f

CMD_SOFT_RESET_REG      = 0xb6

CMD_PMU_ACC_SUSPEND     = 0x10
CMD_PMU_ACC_NORMAL      = 0x11
CMD_PMU_ACC_LP1         = 0x12
CMD_PMU_ACC_LP2         = 0x13
CMD_PMU_GYRO_SUSPEND    = 0x14
CMD_PMU_GYRO_NORMAL     = 0x15
CMD_PMU_GYRO_FASTSTART  = 0x17

BMI160_USER_DATA_14_ADDR = 0X12 # accel x 
BMI160_USER_DATA_15_ADDR = 0X13 # accel x 
BMI160_USER_DATA_16_ADDR = 0X14 # accel y 
BMI160_USER_DATA_17_ADDR = 0X15 # accel y 
BMI160_USER_DATA_18_ADDR = 0X16 # accel z 
BMI160_USER_DATA_19_ADDR = 0X17 # accel z 

BMI160_USER_DATA_8_ADDR  = 0X0C # gyro x 
BMI160_USER_DATA_9_ADDR  = 0X0D # gyro x 
BMI160_USER_DATA_10_ADDR = 0X0E # gyro y 
BMI160_USER_DATA_11_ADDR = 0X0F # gyro y
BMI160_USER_DATA_12_ADDR = 0X10 # gyro z
BMI160_USER_DATA_13_ADDR = 0X11 # gyro z

BMI160_RA_FOC_CONF       = 0x69
BMI160_STATUS_FOC_RDY    = 3
BMI160_RA_STATUS         = 0x1B
BMI160_CMD_START_FOC     = 0x03

BMI160_FOC_ACC_X_BIT     = 4
BMI160_FOC_ACC_Y_BIT     = 2
BMI160_FOC_ACC_Z_BIT     = 0

BMI160_RA_OFFSET_0       = 0x71
BMI160_RA_OFFSET_1       = 0x72
BMI160_RA_OFFSET_2       = 0x73
BMI160_RA_OFFSET_3       = 0x74
BMI160_RA_OFFSET_4       = 0x75
BMI160_RA_OFFSET_5       = 0x76
BMI160_RA_OFFSET_6       = 0x77

BMI160_FOC_GYR_EN        = 6
BMI160_ACC_OFFSET_EN     = 6
BMI160_GYR_OFFSET_EN     = 7

#Global data
acc_x = 0
acc_y = 0
acc_z = 0
acc_x_raw = 0
acc_y_raw = 0
acc_z_raw = 0

gyro_x = 0
gyro_y = 0
gyro_z = 0
gyro_x_raw = 0
gyro_y_raw = 0
gyro_z_raw = 0

x_accelOffset = 0
y_accelOffset = 0
z_accelOffset = 0

class accel_gyro_dev(): 

  def chip_init(self):

    chipid = bus.read_byte_data(BMI160_DEVICE_ADDRESS, BMI160_REGA_USR_CHIP_ID)

    print("---------")
    if chipid == 0xD1 :
      print("chip id is 0x%X, BMI160" % chipid)
    else :
      print("Exit")
      sys.exit()
    print("---------")

    #chip init
    bus.write_byte_data(BMI160_DEVICE_ADDRESS, BMI160_REGA_USR_ACC_CONF_ADDR, 0x28)
    bus.write_byte_data(BMI160_DEVICE_ADDRESS, BMI160_REGA_USR_ACC_RANGE_ADDR, 0x3)
    bus.write_byte_data(BMI160_DEVICE_ADDRESS, BMI160_REGA_USR_GYR_CONF_ADDR, 0x28)
    bus.write_byte_data(BMI160_DEVICE_ADDRESS, BMI160_REGA_USR_GYR_RANGE_ADDR, 0x0)

    #command register
    bus.write_byte_data(BMI160_DEVICE_ADDRESS, BMI160_REGA_CMD_CMD_ADDR, CMD_SOFT_RESET_REG)


  def reg_read_bits(self,reg, pos, len):
    b = bus.read_byte_data(BMI160_DEVICE_ADDRESS, reg)
    mask = (1 << len) - 1
    b >>= pos
    b &= mask
    return b;

  def reg_write_bits(self,reg, data, pos, len):
    b = bus.read_byte_data(BMI160_DEVICE_ADDRESS, reg)
    mask = ((1 << len) - 1) << pos
    data <<= pos; # shift data into correct position
    data &= mask; # zero all non-important bits in data
    b &= ~(mask); # zero all important bits in existing byte
    b |= data; # combine data with existing byte
    bus.write_byte_data(BMI160_DEVICE_ADDRESS, reg, b)

  def enable_accel(self):
    #op_mode set to 0 and go to normal mode
    sleep(0.1)
    bus.write_byte_data(BMI160_DEVICE_ADDRESS, BMI160_REGA_CMD_CMD_ADDR, CMD_PMU_ACC_NORMAL)
    sleep(0.1)
    return;

  def read_accel(self):
    acc_value = [ 0, 0, 0, 0, 0, 0]
    
    global acc_x
    global acc_y
    global acc_z
    
    global acc_x_raw
    global acc_y_raw
    global acc_z_raw

    global x_accelOffset
    global y_accelOffset
    global z_accelOffset

    #read acc xyz
    acc_value = bus.read_i2c_block_data(BMI160_DEVICE_ADDRESS, BMI160_USER_DATA_14_ADDR, 6)
        
    acc_x_raw =  (acc_value[1] << 8) | acc_value[0]
    acc_y_raw =  (acc_value[3] << 8) | acc_value[2]
    acc_z_raw =  (acc_value[5] << 8) | acc_value[4]

    if(acc_x_raw > 0x7fff) :
      acc_x_raw = -(0xffff - acc_x_raw + 1) 

    if(acc_y_raw > 0x7fff) :
      acc_y_raw = -(0xffff - acc_y_raw + 1) 

    if(acc_z_raw > 0x7fff) :
      acc_z_raw = -(0xffff - acc_z_raw + 1) 
              
    acc_x = (acc_x_raw * 9.8) / (0x8000 / 2)
    acc_y = (acc_y_raw * 9.8) / (0x8000 / 2)
    acc_z = (acc_z_raw * 9.8) / (0x8000 / 2)

    return;

  def enable_gyro(self):
    #op_mode set to 0 and go to normal mode
    sleep(0.1)
    bus.write_byte_data(BMI160_DEVICE_ADDRESS, BMI160_REGA_CMD_CMD_ADDR, CMD_PMU_GYRO_NORMAL)
    sleep(0.1)

    return;

  def read_gyro(self):
    gyro_value = [ 0, 0, 0, 0, 0, 0]
    global gyro_x
    global gyro_y
    global gyro_z

    global gyro_x_raw
    global gyro_y_raw
    global gyro_z_raw

    #read gyro xyz
    gyro_value = bus.read_i2c_block_data(BMI160_DEVICE_ADDRESS, BMI160_USER_DATA_8_ADDR, 6)

    gyro_x_raw =  (gyro_value[1] << 8) | gyro_value[0]
    gyro_y_raw =  (gyro_value[3] << 8) | gyro_value[2]
    gyro_z_raw =  (gyro_value[5] << 8) | gyro_value[4]
        
    if(gyro_x_raw > 0x7fff) :
      gyro_x_raw = -(0xffff - gyro_x_raw + 1) 

    if(gyro_y_raw > 0x7fff) :
      gyro_y_raw = -(0xffff - gyro_y_raw + 1) 

    if(gyro_z_raw > 0x7fff) :
      gyro_z_raw = -(0xffff - gyro_z_raw + 1) 

    gyro_x = (gyro_x_raw * 2000) / 0x8000
    gyro_y = (gyro_y_raw * 2000) / 0x8000
    gyro_z = (gyro_z_raw * 2000) / 0x8000

    return;

  def getAccelOffsetEnabled(self):
    accelStatus = self.reg_read_bits(BMI160_RA_OFFSET_6 , BMI160_ACC_OFFSET_EN, 1)
    print("status %x" %( accelStatus ))
    return;

  def setAccelOffsetEnabled(self,enabled):
    self.reg_write_bits(BMI160_RA_OFFSET_6, enabled, BMI160_ACC_OFFSET_EN, 1)
    return;

  def autoCalibrateXAccelOffset(self,target):
    if (target == 1):
      foc_conf = (0x1 << BMI160_FOC_ACC_X_BIT);
    elif (target == -1):
      foc_conf = (0x2 << BMI160_FOC_ACC_X_BIT);
    elif (target == 0):
      foc_conf = (0x3 << BMI160_FOC_ACC_X_BIT);
    else:
      return;  #Invalid target value 
    
    bus.write_byte_data(BMI160_DEVICE_ADDRESS, BMI160_RA_FOC_CONF, foc_conf)
    bus.write_byte_data(BMI160_DEVICE_ADDRESS, BMI160_REGA_CMD_CMD_ADDR, BMI160_CMD_START_FOC)

    while True:
      ra_status = bus.read_byte_data(BMI160_DEVICE_ADDRESS, BMI160_RA_STATUS)
      if((ra_status & 0x08) != 0):
        break
    return;

  def autoCalibrateYAccelOffset(self,target):
    if (target == 1):
      foc_conf = (0x1 << BMI160_FOC_ACC_Y_BIT);
    elif (target == -1):
      foc_conf = (0x2 << BMI160_FOC_ACC_Y_BIT);
    elif (target == 0):
      foc_conf = (0x3 << BMI160_FOC_ACC_Y_BIT);
    else:
      return;  #Invalid target value 
    
    bus.write_byte_data(BMI160_DEVICE_ADDRESS, BMI160_RA_FOC_CONF, foc_conf)
    bus.write_byte_data(BMI160_DEVICE_ADDRESS, BMI160_REGA_CMD_CMD_ADDR, BMI160_CMD_START_FOC)

    while True:
      ra_status = bus.read_byte_data(BMI160_DEVICE_ADDRESS, BMI160_RA_STATUS)
      if((ra_status & 0x08) != 0):
        break
    return;

  def autoCalibrateZAccelOffset(self,target):
    if (target == 1):
      foc_conf = (0x1 << BMI160_FOC_ACC_Z_BIT);
    elif (target == -1):
      foc_conf = (0x2 << BMI160_FOC_ACC_Z_BIT);
    elif (target == 0):
      foc_conf = (0x3 << BMI160_FOC_ACC_Z_BIT);
    else:
      return;  #Invalid target value 
    
    bus.write_byte_data(BMI160_DEVICE_ADDRESS, BMI160_RA_FOC_CONF, foc_conf)
    bus.write_byte_data(BMI160_DEVICE_ADDRESS, BMI160_REGA_CMD_CMD_ADDR, BMI160_CMD_START_FOC)

    while True:
      ra_status = bus.read_byte_data(BMI160_DEVICE_ADDRESS, BMI160_RA_STATUS)
      if((ra_status & 0x8) != 0):
        break
    return;

  def getAccelOffset(self):
    global x_accelOffset
    global y_accelOffset
    global z_accelOffset

    print("Internal sensor offsets AFTER calibration...")
    x_accelOffset = bus.read_byte_data(BMI160_DEVICE_ADDRESS, BMI160_RA_OFFSET_0)
    y_accelOffset = bus.read_byte_data(BMI160_DEVICE_ADDRESS, BMI160_RA_OFFSET_1)
    z_accelOffset = bus.read_byte_data(BMI160_DEVICE_ADDRESS, BMI160_RA_OFFSET_2)

    if(x_accelOffset > 0x7f) :
      x_accelOffset = -(0xff - x_accelOffset + 1)

    if(y_accelOffset > 0x7f) :
      y_accelOffset = -(0xff - y_accelOffset + 1)

    if(z_accelOffset > 0x7f) :
      z_accelOffset = -(0xff - z_accelOffset + 1)

    print("x_accelOffset %d y_accelOffset %d z_accelOffset %d" % (x_accelOffset, y_accelOffset, z_accelOffset))
    return;

  def autoCalibrateGyroOffset(self):
    foc_conf = (1 << BMI160_FOC_GYR_EN);
    bus.write_byte_data(BMI160_DEVICE_ADDRESS, BMI160_RA_FOC_CONF, foc_conf)
    bus.write_byte_data(BMI160_DEVICE_ADDRESS, BMI160_REGA_CMD_CMD_ADDR, BMI160_CMD_START_FOC) 
    
    while True:
      ra_status = bus.read_byte_data(BMI160_DEVICE_ADDRESS, BMI160_RA_STATUS)
      if((ra_status & 0x8) != 0):
        break
    return;

  def sign_extend(self,value, bits):
      sign_bit = 1 << (bits - 1)
      return (value & (sign_bit - 1)) - (value & sign_bit)

  def getGyroOffset(self):
    x_offset = bus.read_byte_data(BMI160_DEVICE_ADDRESS, BMI160_RA_OFFSET_3) 
    x_offset |=  (self.reg_read_bits(BMI160_RA_OFFSET_6, 0, 2)) << 8  #Get OFFSET_6 bit 0 bit 1 for off_gry_x<9:8>
    x_gyroOffset = self.sign_extend(x_offset, 10)

    y_offset = bus.read_byte_data(BMI160_DEVICE_ADDRESS, BMI160_RA_OFFSET_4)
    y_offset |=  (self.reg_read_bits(BMI160_RA_OFFSET_6, 2, 2)) << 8 #Get OFFSET_6 bit 2 bit 3 for off_gry_y<9:8>
    y_gyroOffset = self.sign_extend(y_offset, 10)

    z_offset = bus.read_byte_data(BMI160_DEVICE_ADDRESS, BMI160_RA_OFFSET_5)
    z_offset |=  (self.reg_read_bits(BMI160_RA_OFFSET_6, 4, 2)) << 8 #Get OFFSET_6 bit 4 bit 5 for off_gry_z<9:8>
    z_gyroOffset = self.sign_extend(z_offset, 10)

    print("x_gyroOffset %d y_gyroOffset %d z_gyroOffset %d" % (x_gyroOffset, y_gyroOffset, z_gyroOffset))

    return;

  def getGyroOffsetEnabled(self):
    gyroStatus = self.reg_read_bits(BMI160_RA_OFFSET_6 , BMI160_GYR_OFFSET_EN, 1)
    print("GyroOffsetEnabled %x" %( gyroStatus ))
    return;

  def setGyroOffsetEnabled(self,enabled):
    self.reg_write_bits(BMI160_RA_OFFSET_6, enabled, BMI160_GYR_OFFSET_EN, 1)
    return;

  def calibration_process(self): 
    self.enable_accel()
    self.enable_gyro()

    
    print("Starting Acceleration calibration and enabling offset compensation...")
    self.autoCalibrateXAccelOffset(0)
    self.autoCalibrateYAccelOffset(0)
    self.autoCalibrateZAccelOffset(1)
    self.getAccelOffset()
    print("Done")
    
    print("Starting Gyroscope calibration and enabling offset compensation...")
    self.autoCalibrateGyroOffset()
    self.getGyroOffset()
    print("Done")
    
    self.setGyroOffsetEnabled(1)
    self.setAccelOffsetEnabled(1)

  def show_accel_gyro(self):
    self.read_gyro()
    self.read_accel()
    print("Apply Platform Matrix")
    print("===============================================================")
    print("gyro x_raw = %d, y_raw = %d z_raw = %d" % (-gyro_x_raw, gyro_y_raw, -gyro_z_raw))
    print("gyro x = %d, y = %d z = %d" % (-gyro_x, gyro_y, -gyro_z))
    print("===============================================================")
    print("accel x_raw = %d, y_raw = %d z_raw = %d" % (-acc_x_raw, acc_y_raw, -acc_z_raw))
    print("accel x = %d, y = %d z = %d" % (-acc_x, acc_y, -acc_z))
    print("===============================================================")
    return [-acc_x, acc_y, -acc_z],[-gyro_x, gyro_y, -gyro_z]

    """""
    try:
      while True:
      
        
        print "==============================================================="
        print "gyro x_raw = %d, y_raw = %d z_raw = %d" % (gyro_x_raw, gyro_y_raw, gyro_z_raw)
        print "gyro x = %d, y = %d z = %d" % (gyro_x, gyro_y, gyro_z)
        print "==============================================================="
        print "accel x_raw = %d, y_raw = %d z_raw = %d" % (acc_x_raw, acc_y_raw, acc_z_raw)
        print "accel x = %d, y = %d z = %d" % (acc_x, acc_y, acc_z)
        print "==============================================================="
        
        self.read_gyro()
        self.read_accel()
        print("Apply Platform Matrix")
        print("===============================================================")
        print("gyro x_raw = %d, y_raw = %d z_raw = %d" % (-gyro_x_raw, gyro_y_raw, -gyro_z_raw))
        print("gyro x = %d, y = %d z = %d" % (-gyro_x, gyro_y, -gyro_z))
        print("===============================================================")
        print("accel x_raw = %d, y_raw = %d z_raw = %d" % (-acc_x_raw, acc_y_raw, -acc_z_raw))
        print("accel x = %d, y = %d z = %d" % (-acc_x, acc_y, -acc_z))
        print("===============================================================")
        sleep(0.5)
        os.system('clear')
    except KeyboardInterrupt:
        print('Exit!')
    return;
    """

if __name__ == '__main__':
  instance = accel_gyro_dev()
  instance.show_accel_gyro()
  sys.exit()




