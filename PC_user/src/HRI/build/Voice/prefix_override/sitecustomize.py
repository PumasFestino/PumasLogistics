import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/vero/PumasLogistics/PC_user/src/HRI/install/Voice'
