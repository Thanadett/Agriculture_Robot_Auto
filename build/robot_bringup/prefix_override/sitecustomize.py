import sys
if sys.prefix == '/home/prukubt/.platformio/penv':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/prukubt/392_Agri/install/robot_bringup'
