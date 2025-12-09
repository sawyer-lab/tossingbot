from dataclasses import dataclass, field
import numpy as np

@dataclass
class PlannerConfig:
    table_height: float = 0.0
    solver_steps: int = 20
    
    max_vel: np.ndarray = field(default_factory=lambda: np.array([1.7, 1.7, 1.7, 2.0, 2.0, 3.0, 3.0]))
    max_acc: np.ndarray = field(default_factory=lambda: np.array([2.5, 2.5, 2.5, 3.0, 3.0, 4.0, 4.0]))
    
    # --- WEIGHTS ---
    w_smooth: float = 0.01     # LOWERED: Allow faster wrist movements
    w_slack: float  = 10000.0  # Keep floor safety high
    w_track: float  = 10.0     
    w_goal: float   = 1000.0   
    w_pos: float    = 2000.0   # High priority on XYZ
    w_ori: float    = 5000.0
    
    # --- ELBOW PROTECTION ---
    # We apply a "One-Sided" penalty.
    # If J1 is positive, cost is 0 (Free movement).
    # If J1 goes below 'min_j1', cost explodes.
    
    w_elbow: float  = 500.0  # Strength of the repulsion
    min_j1: float   = -0.2   # The "Virtual Wall" for Joint 1