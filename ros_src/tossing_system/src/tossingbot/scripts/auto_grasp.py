#!/usr/bin/env python3.8
import sys
import os
import argparse

# --- 1. PYTHON 3 ROS COMPATIBILITY ---
sys.path.insert(0, '/geometry2_ws/devel/lib/python3/dist-packages')
sys.path.insert(0, '/cv_bridge_ws/devel/lib/python3/dist-packages')

# --- 2. CRITICAL IMPORT ORDER FIX ---
try:
    import open3d as o3d
except ImportError:
    print("Warning: Could not import open3d early. This might cause crashes.")

import torch 
import cv2   
import rospy
import numpy as np
import random
import math

# --- 3. MODULE IMPORTS ---
from tossingbot import config as cfg
from tossingbot.environment.tossing_env import TossingEnv
from tossingbot.learning.agent import TossingAgent
from tossingbot.learning.utils import RotationTransformer
from tossingbot.learning.logger import TrainingLogger
from tossingbot.learning.session_manager import SessionManager

# =============================================================================
# DEBUG MODE: Set to False for normal training/execution
# =============================================================================
DEBUG_MODE = False  # Set to True for visual debugging with user confirmation

def parse_args():
    """Parse command line arguments"""
    parser = argparse.ArgumentParser(
        description='TossingBot Auto-Grasp Training and Demo System',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog='''
Session Workflow:
  1. Start training with interactive session selection
  2. Training continues from last checkpoint automatically
  3. Buffer persists - no loss of experience across restarts
  4. Analyze results with session_tools.py
  5. Test models in demo mode

Examples:
  # Start training (interactive session selection)
  %(prog)s --mode training
  
  # Train on specific objects
  %(prog)s --mode training --train-objects L_shape cube cylinder
  
  # Continue specific session
  %(prog)s --mode training --session session_baseline_v1
  
  # Evaluation mode: test on different objects
  %(prog)s --eval-only --session session_baseline --eval-objects puck bolt C_shape
  
  # Demo a trained model (interactive selection)
  %(prog)s --mode demo
  
  # List all sessions
  %(prog)s --list

Analysis & Visualization:
  Use session_tools.py for analysis and visualization:
  
    # Analyze session
    python session_tools.py analyze --session <session_id>
    
    # Compare experiments
    python session_tools.py compare --sessions <id1> <id2>

Experiments:
  Use run_experiment.sh to run complete train/test experiments:
  
    ./run_experiment.sh experiments/exp_baseline.yaml
        '''
    )
    parser.add_argument('--mode', type=str, default='training', choices=['training', 'demo'],
                        help='Mode: training (learn) or demo (inference only)')
    parser.add_argument('--session', type=str, default=None,
                        help='Session ID to load (skip interactive selection)')
    parser.add_argument('--no-buffer', action='store_true',
                        help='Start with empty replay buffer (training mode only)')
    parser.add_argument('--list', action='store_true',
                        help='List available sessions and exit')
    parser.add_argument('--debug', action='store_true',
                        help='Enable debug mode with visual confirmation')
    
    # Train/Test Split Arguments (EXPLICIT OBJECT SPECIFICATION)
    parser.add_argument('--train-objects', nargs='+', default=None,
                        help='Objects for training (e.g., --train-objects L_shape cube cylinder)')
    parser.add_argument('--eval-only', action='store_true',
                        help='Evaluation mode: inference only, no training')
    parser.add_argument('--eval-objects', nargs='+', default=None,
                        help='Objects for evaluation (e.g., --eval-objects puck bolt C_shape)')
    parser.add_argument('--eval-episodes', type=int, default=100,
                        help='Number of episodes for evaluation (default: 100)')
    
    return parser.parse_args()

def get_epsilon(step, inference_only):
    if inference_only: return 0.0
    if step >= cfg.EXPLORE_STEPS: return cfg.EXPLORE_END
    frac = float(step) / cfg.EXPLORE_STEPS
    return cfg.EXPLORE_START - frac * (cfg.EXPLORE_START - cfg.EXPLORE_END)

def main():
    # Parse arguments before ROS init
    args = parse_args()
    
    # Update global flags from args
    global DEBUG_MODE
    if args.debug:
        DEBUG_MODE = True
    
    # Initialize session manager
    session_manager = SessionManager(cfg.SESSION_BASE_DIR)
    
    # List sessions if requested
    if args.list:
        print("\n=== Training Sessions ===")
        training_sessions = session_manager.list_sessions("training")
        for session in training_sessions:
            print(f"  - {session}")
        print("\n=== Demo Sessions ===")
        demo_sessions = session_manager.list_sessions("demo")
        for session in demo_sessions:
            print(f"  - {session}")
        return
    
    # Select or load session
    if args.session:
        session = session_manager.load_session(args.session, args.mode)
        if session is None:
            # If session doesn't exist and we're in training mode, create it
            if args.mode == "training":
                print(f"Creating new session: {args.session}")
                session = session_manager.create_session(args.mode, name=args.session)
            else:
                print(f"Error: Could not load session {args.session}")
                return
    else:
        if args.mode == "demo":
            # Demo mode returns (demo_session, training_session)
            result = session_manager.select_session_interactive("demo")
            if isinstance(result, tuple):
                session, training_session = result
            else:
                print("Error: Could not select session")
                return
        else:
            session = session_manager.select_session_interactive("training")
    
    # Determine if inference only
    INFERENCE_ONLY = (args.mode == "demo")
    
    # Initialize ROS
    rospy.init_node('tossingbot_brain')
    
    # Print configuration
    print("\n" + "="*70)
    print(f"TossingBot - {args.mode.capitalize()} Mode")
    print("="*70)
    print(f"Session: {session.name}")
    print(f"Session ID: {session.session_id}")
    print(f"Created: {session.metadata.get('created_at', 'unknown')[:19]}")
    print(f"Last run: {session.metadata.get('last_run', 'unknown')[:19]}")
    if not INFERENCE_ONLY:
        print(f"Total steps: {session.metadata.get('total_steps', 0)}")
        print(f"Success rate: {session.metadata.get('success_rate', 0.0)*100:.1f}%")
    print(f"Debug Mode: {DEBUG_MODE}")
    print(f"Training: {not INFERENCE_ONLY}")
    print("="*70 + "\n")

    # Determine which objects to use
    train_objects = args.train_objects if args.train_objects else cfg.DEFAULT_TRAIN_OBJECTS
    eval_objects = args.eval_objects if args.eval_objects else train_objects
    
    # Log object configuration
    print(f"Objects to use: {', '.join(train_objects if not args.eval_only else eval_objects)}")
    if args.eval_only and eval_objects != train_objects:
        print(f"  (Training was on: {', '.join(train_objects)})")
    print()

    # 1. Init Environment and Agent
    # Use eval_objects if eval-only mode, otherwise use train_objects
    objects_for_env = eval_objects if args.eval_only else train_objects
    env = TossingEnv(allowed_objects=objects_for_env)
    
    # In demo mode, load weights from training session
    if args.mode == "demo":
        checkpoint_type = session.metadata.get('source_checkpoint', 'best')
        checkpoint_name = f"checkpoint_{checkpoint_type}.pth"
        
        # Create agent with training session for loading weights
        temp_session = session_manager.load_session(training_session.session_id, "training")
        agent = TossingAgent(temp_session, load_buffer=False, load_weights=True)
        # But use demo session for logging
        session_for_logging = session
    else:
        # Training mode
        load_buf = not args.no_buffer
        agent = TossingAgent(session, load_buffer=load_buf, load_weights=True)
        session_for_logging = session
        
        # Store train/test object configuration in metadata
        if train_objects:
            session.metadata['train_objects'] = train_objects
        if eval_objects and eval_objects != train_objects:
            session.metadata['test_objects'] = eval_objects
        session.save_metadata()
    
    transformer = RotationTransformer() 
    cv2.namedWindow("Dashboard", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Dashboard", 1400, 900)
    
    # Initialize logger (with train_objects for seen/unseen tracking)
    logger = TrainingLogger(session_for_logging, train_objects=train_objects) if not INFERENCE_ONLY else None
    
    # 2. Reset
    obs, _ = env.reset(force_new=True)
    failed_attempts = [] # Short term memory
    
    step_count = session.metadata.get('total_steps', 0)  # Continue from last step
    episode_count = session.metadata.get('total_episodes', 0)
    episode_steps = 0
    episode_successes = 0
    
    rospy.loginfo("STARTING MAIN LOOP...")

    # 3. Loop
    while not rospy.is_shutdown():
        if obs is None: 
            obs = env.get_observation()
            rospy.sleep(0.1); continue

        # If all current objects are picked, reset the scene and skip this iteration
        if env.sim.all_objects_picked():
            rospy.loginfo("All objects picked in current scene. Resetting environment...")
            obs, _ = env.reset(force_new=True) # Force a new scene
            continue # Skip action generation for this empty step

        # --- A. THINK ---
        eps = get_epsilon(step_count, INFERENCE_ONLY)
        # agent returns the specific rotation index and the pixel IN THAT ROTATED FRAME
        # In DEBUG_MODE, disable failed_attempts inhibition to test all rotations
        fail_list = [] if DEBUG_MODE else failed_attempts
        rot_idx, u_rot, v_rot, debug = agent.get_action(obs, eps, failed_attempts=fail_list)
        
        # --- B. TRANSFORM (Pixel Frame -> World Frame) ---
        # Image was rotated by (TOTAL_DEG/NUM_ROTATIONS)*rot_idx
        # The rotation code internally handles the negation for affine_grid
        angle_deg = (cfg.TOTAL_DEG / cfg.NUM_ROTATIONS) * rot_idx
        
        u_world, v_world = transformer.rotate_pixel(
            u_rot, v_rot, angle_deg, cfg.IMG_H, cfg.IMG_W, to_gripper_frame=False
        )

        # --- C. VISUALIZE (BEFORE ACTING) ---
        # We pass everything needed to draw the full "Thought Process"
        viz_img = render_dashboard(obs, debug, rot_idx, u_rot, v_rot, u_world, v_world, angle_deg, failed_attempts)
        cv2.imshow("Dashboard", viz_img)
        
        # DEBUG_MODE: Wait for user confirmation before executing
        if DEBUG_MODE:
            print(f"\n{'='*70}")
            print(f"[Step {step_count}] {debug['type']} (Eps: {eps:.2f})")
            print(f"  Rotation: {rot_idx} → Image rotated by {angle_deg:.1f}°")
            print(f"  Selected in ROTATED image: (u={u_rot}, v={v_rot})")
            print(f"  Transformed to WORLD image: (u={u_world}, v={v_world})")
            print(f"  (Should align visually: GREEN crosshair = CYAN crosshair location)")
            print(f"{'='*70}")
            print("Press ENTER to execute grasp (or 'q' to quit)...")
            
            key = cv2.waitKey(0)
            if key == ord('q'):
                break
        else:
            # Normal mode: just brief waitKey for cv2.imshow to update
            cv2.waitKey(1)

        # --- D. ACT ---
        action_type = "EXPLORE" if eps > random.random() else "EXPLOIT"
        
        # Get object poses BEFORE action (for logging)
        object_poses = env.sim.get_object_poses()
        
        # Invert rotation index for the planner to test for a convention mismatch
        rot_idx_for_planner = (cfg.NUM_ROTATIONS - rot_idx) % cfg.NUM_ROTATIONS if rot_idx != 0 else 0

        print(f"\n[Step {step_count:05d}] {action_type} | Rot={rot_idx} (Planner_Rot={rot_idx_for_planner}) | Pixel=({u_world},{v_world}) | Eps={eps:.3f}")
        reward, picked_object = env.step(u_world, v_world, rot_idx_for_planner)
        
        # --- E. LEARN ---
        # The original rot_idx is pushed to the buffer
        if not INFERENCE_ONLY:
            agent.buffer.push(obs.cpu(), u_rot, v_rot, rot_idx, reward)
            loss = agent.train()
        else:
            loss = 0.0
        
        # --- F. LOG ---
        success = (reward > 0.5)
        if logger:
            step_data = {
                'step': step_count,
                'episode': episode_count,
                'object_name': picked_object if picked_object else 'unknown',
                'object_poses': object_poses,
                'predicted_grasp': {
                    'u': int(u_world),
                    'v': int(v_world),
                    'u_rot': int(u_rot),
                    'v_rot': int(v_rot),
                    'rotation_idx': int(rot_idx),
                    'angle_deg': float(angle_deg),
                    'confidence': float(debug['conf'])
                },
                'action_type': action_type,
                'epsilon': float(eps),
                'reward': float(reward),
                'loss': float(loss),
                'success': success,
                'objects_in_scene': len(object_poses),
                'failed_attempts_count': len(failed_attempts)
            }
            logger.log_step(step_data)
        
        # Track episode statistics
        episode_steps += 1
        if success:
            episode_successes += 1
        
        # --- G. SAVE ---
        if step_count % cfg.SAVE_INTERVAL == 0 and step_count > 0 and not INFERENCE_ONLY:
            success_rate = logger.get_current_success_rate() if logger else 0.0
            agent.save_snapshot(step_count, success_rate)
            if logger:
                logger.flush()  # Flush logs at checkpoint intervals

        # --- H. RESULT ---
        status = "SUCCESS" if success else "FAIL"
        print(f"   Result: {status} | Reward={reward:.1f} | Loss={loss:.4f}\n")

        if success:
            failed_attempts = []
        else:
            failed_attempts.append((rot_idx, u_rot, v_rot))
        
        # --- I. NEXT EPISODE ---
        obs, is_new = env.reset(previous_success=success)
        if is_new: 
            # Log episode end
            if logger and episode_steps > 0:
                episode_summary = {
                    'episode': episode_count,
                    'steps_in_episode': episode_steps,
                    'successes_in_episode': episode_successes,
                    'success_rate': episode_successes / episode_steps if episode_steps > 0 else 0.0
                }
                logger.log_episode_end(episode_summary)
            
            failed_attempts = []
            episode_count += 1
            episode_steps = 0
            episode_successes = 0
            
        step_count += 1
    
    # Cleanup on exit
    if logger:
        logger.close()
        print("\nTraining session complete. Logs saved.")

def render_dashboard(obs_tensor, debug, chosen_rot, u_rot, v_rot, u_world, v_world, angle_deg, failures):
    """
    Stitches a comprehensive dashboard with detailed visual debugging
    [ Original | Rot0 | Rot1 | Rot2 | Rot3 ]
    [ Overlay showing transformation ]
    [      ORIGINAL WORLD FRAME VIEW      ]
    """
    # Helper: Convert PyTorch tensor (C,H,W) to BGR Image
    def to_cv(t):
        if t.dim() == 4: t = t[0] # Handle batch dim
        img = t.detach().cpu().numpy()[:3].transpose(1, 2, 0)
        img = np.clip(img, 0, 1) * 255
        return cv2.cvtColor(img.astype(np.uint8), cv2.COLOR_RGB2BGR)
    
    def draw_gripper_frame(img, label=""):
        """Draw coordinate axes showing gripper/physical frame (ROS convention)"""
        if not DEBUG_MODE:
            return img
        img = img.copy()
        center_y = img.shape[0] // 2
        center_x = img.shape[1] // 2
        line_len = 25
        
        # RED: +X axis (forward)
        cv2.arrowedLine(img, (center_x, center_y), (center_x + line_len, center_y), 
                       (0, 0, 255), 2, tipLength=0.3)
        # GREEN: +Y axis (left)
        cv2.arrowedLine(img, (center_x, center_y), (center_x, center_y - line_len), 
                       (0, 255, 0), 2, tipLength=0.3)
        # BLUE: +Z axis (up, out of image plane)
        cv2.circle(img, (center_x, center_y), 3, (255, 0, 0), -1)
        
        if label:
            cv2.putText(img, label, (5, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
        
        return img

    # --- 1. TOP ROW: All 4 Rotations ---
    row_images = []
    
    # Get original image for overlay comparison
    orig_img_cv = to_cv(debug['state_viz'])
    
    for i in range(cfg.NUM_ROTATIONS):
        angle = (cfg.TOTAL_DEG / cfg.NUM_ROTATIONS) * i
        
        # Get what the network actually sees
        network_input_tensor = debug['inputs'][i]
        network_input = to_cv(network_input_tensor)
        
        # DEBUG MODE: Create overlay (original=green tint, rotated=magenta tint)
        if DEBUG_MODE:
            orig_tinted = orig_img_cv.copy()
            orig_tinted[:,:,1] = np.clip(orig_tinted[:,:,1] * 1.3, 0, 255)  # Green boost
            
            rotated_tinted = network_input.copy()
            rotated_tinted[:,:,0] = np.clip(rotated_tinted[:,:,0] * 1.3, 0, 255)  # Blue boost
            rotated_tinted[:,:,2] = np.clip(rotated_tinted[:,:,2] * 1.3, 0, 255)  # Red boost
            
            # Blend 50/50 to show transformation
            img_to_show = cv2.addWeighted(orig_tinted, 0.5, rotated_tinted, 0.5, 0)
            
            # Draw gripper frame on overlay
            img_to_show = draw_gripper_frame(img_to_show)
        else:
            img_to_show = network_input
        
        # Extract and visualize Depth Channel Input
        depth_channel_tensor = network_input_tensor[3] # Assuming 4th channel (0-indexed) is depth
        depth_numpy = depth_channel_tensor.detach().cpu().numpy()
        
        # Robustly scale normalized values to 0-255 for visualization
        # Clip to 1st and 99th percentile to handle outliers
        p1, p99 = np.percentile(depth_numpy, [1, 99])
        depth_clipped = np.clip(depth_numpy, p1, p99)

        # Scale the clipped data to 0-1
        if p99 - p1 > 1e-6:
            depth_normalized_for_viz = (depth_clipped - p1) / (p99 - p1)
        else:
            depth_normalized_for_viz = np.zeros_like(depth_clipped)

        depth_viz_cv = (depth_normalized_for_viz * 255).astype(np.uint8)
        depth_viz_cv = cv2.cvtColor(depth_viz_cv, cv2.COLOR_GRAY2BGR) # Convert to BGR for hstack
        cv2.putText(depth_viz_cv, "Depth Input", (5, 15), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)

        # Add angle label
        cv2.putText(img_to_show, f"{angle:.0f}deg", (5, img_to_show.shape[0]-5), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
        
        # Heatmap
        heatmap_raw = torch.sigmoid(debug['heatmaps'][i, 0]).detach().cpu().numpy()
        heatmap_color = cv2.applyColorMap((heatmap_raw * 255).astype(np.uint8), cv2.COLORMAP_JET)
        
        # Combine (Input RGB + Input Depth + Heatmap) side-by-side
        pair = np.hstack([img_to_show, depth_viz_cv, heatmap_color])
        
        # Highlight chosen rotation
        if i == chosen_rot:
            cv2.rectangle(pair, (0,0), (pair.shape[1]-1, pair.shape[0]-1), (0, 255, 0), 3)
            
            # Crosshair on selected point
            cv2.line(pair, (v_rot-10, u_rot), (v_rot+10, u_rot), (0, 255, 0), 1)
            cv2.line(pair, (v_rot, u_rot-10), (v_rot, u_rot+10), (0, 255, 0), 1)
            cv2.circle(pair, (v_rot, u_rot), 2, (0, 255, 0), -1)
            
            # Same on heatmap and depth viz
            offset_rgb = img_to_show.shape[1]
            offset_depth = depth_viz_cv.shape[1]
            
            # On depth
            cv2.line(pair, (v_rot+offset_rgb-10, u_rot), (v_rot+offset_rgb+10, u_rot), (0, 255, 0), 1)
            cv2.line(pair, (v_rot+offset_rgb, u_rot-10), (v_rot+offset_rgb, u_rot+10), (0, 255, 0), 1)
            cv2.circle(pair, (v_rot + offset_rgb, u_rot), 2, (0, 255, 0), -1)

            # On heatmap
            cv2.line(pair, (v_rot+offset_rgb+offset_depth-10, u_rot), (v_rot+offset_rgb+offset_depth+10, u_rot), (0, 255, 0), 1)
            cv2.line(pair, (v_rot+offset_rgb+offset_depth, u_rot-10), (v_rot+offset_rgb+offset_depth, u_rot+10), (0, 255, 0), 1)
            cv2.circle(pair, (v_rot + offset_rgb + offset_depth, u_rot), 2, (0, 255, 0), -1)

        row_images.append(pair)

    # Stack all horizontally
    top_row = np.hstack(row_images)

    # --- 2. BOTTOM ROW: World View ---
    world_img = to_cv(obs_tensor)
    
    # Draw gripper frame on world view too
    world_img = draw_gripper_frame(world_img, "WORLD VIEW")
    
    # Scale world image to match top row width
    scale = top_row.shape[1] / world_img.shape[1]
    new_h = int(world_img.shape[0] * scale)
    world_img_resized = cv2.resize(world_img, (top_row.shape[1], new_h))
    
    # Since we resized, we must scale the coordinates too
    u_world_sc = int(u_world * scale)
    v_world_sc = int(v_world * scale)
    
    # FINER CROSSHAIR for world position
    cv2.line(world_img_resized, (v_world_sc-20, u_world_sc), (v_world_sc+20, u_world_sc), (0, 255, 255), 2)
    cv2.line(world_img_resized, (v_world_sc, u_world_sc-20), (v_world_sc, u_world_sc+20), (0, 255, 255), 2)
    cv2.circle(world_img_resized, (v_world_sc, u_world_sc), 3, (0, 255, 255), -1)
    
    # Draw Orientation Arrow
    rad = np.deg2rad(angle_deg) 
    arrow_len = 50 * scale
    end_v = int(v_world_sc + arrow_len * np.cos(rad))
    end_u = int(u_world_sc + arrow_len * np.sin(rad))
    cv2.arrowedLine(world_img_resized, (v_world_sc, u_world_sc), (end_v, end_u), (255, 0, 255), 2, tipLength=0.3)
    
    # Add text labels
    cv2.putText(world_img_resized, f"Rot{chosen_rot} ({angle_deg:.0f}deg) @ ({u_world},{v_world})", 
                (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    cv2.putText(world_img_resized, f"CYAN=Target, MAGENTA=Orientation", 
                (20, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

    # --- 3. COMBINE ---
    final_dashboard = np.vstack([top_row, world_img_resized])
    return final_dashboard

if __name__ == '__main__':
    main()