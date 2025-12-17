import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/vboxuser/Documents/projet_M2_SSIO_2025-2026/src/install/planner'
