from dataclasses import dataclass, field
import numpy as np
import config as cfg

@dataclass
class PlannerConfig:
    table_height: float = cfg.TABLE_HEIGHT
    solver_steps: int = 20  # Back to original
    
    # Weights
    w_smooth: float = 1.0      
    w_slack: float  = 10000.0  
    w_track: float  = 10.0     
    w_goal: float   = 5000.0   
    w_pos: float    = 5000.0   
    w_ori: float    = 5000.0   
    w_reg: float    = 0.0      # Set to 0.0 for experiment
    q_natural: np.ndarray = field(default_factory=lambda: np.array(cfg.NEUTRAL_JOINT_POS))

    # Robot Limits
    min_j1: float   = -0.2   
    max_vel: np.ndarray = field(default_factory=lambda: np.array([1.7, 1.7, 1.7, 2.0, 2.0, 3.0, 3.0]))
    max_acc: np.ndarray = field(default_factory=lambda: np.array([2.5, 2.5, 2.5, 3.0, 3.0, 4.0, 4.0]))