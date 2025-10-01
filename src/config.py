

from enum import IntEnum
    
EffectorId = IntEnum('EffectorId', [('RR_FOOT', 0), ('RL_FOOT', 1), ('FL_FOOT', 2),('FR_FOOT',3)])  
EFFECTOR_NAMES = ['RR_foot_joint', 'RL_foot_joint', 'FL_foot_joint', 'FR_foot_joint']
ROOT_NAME = ['']