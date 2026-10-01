import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/fontan/aruco-localisation-lowlight/aruco-localisation-lowlight/install/aruco_localisation'
