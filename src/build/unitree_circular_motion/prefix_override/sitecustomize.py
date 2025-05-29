import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/drone/comp0244_ws/comp0244-go2/src/install/unitree_circular_motion'
