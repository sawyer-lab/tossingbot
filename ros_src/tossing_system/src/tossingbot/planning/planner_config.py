from dataclasses import dataclass, field
import numpy as np
from tossingbot import config as cfg  # <--- IMPORT THE TRUTH

@dataclass
class PlannerConfig:
    table_height: float = cfg.TABLE_HEIGHT
    solver_steps: int = 40  # Increased from 20 for better resolution
    
    # Weights
    w_smooth: float = 0.01     
    w_slack: float  = 10000.0  
    w_track: float  = 10.0     
    w_goal: float   = 1000.0   
    w_pos: float    = 10000.0   
    w_ori: float    = 10000.0
    w_reg: float    = 0.001    # Reduced further to minimize interference
    q_natural: np.ndarray = field(default_factory=lambda: np.array(cfg.NEUTRAL_JOINT_POS))

    # Robot Limits
    min_j1: float   = -0.2   
    max_vel: np.ndarray = field(default_factory=lambda: np.array([1.7, 1.7, 1.7, 2.0, 2.0, 3.0, 3.0]))
    max_acc: np.ndarray = field(default_factory=lambda: np.array([2.5, 2.5, 2.5, 3.0, 3.0, 4.0, 4.0]))