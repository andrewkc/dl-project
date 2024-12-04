import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/andrewkc/r2_ws/install/obstacle_avoidance/install/obstacle_avoidance'
