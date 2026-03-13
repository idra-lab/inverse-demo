import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/leo/inverse_ws/src/inverse-demo/install/inverse_orchestrator'
