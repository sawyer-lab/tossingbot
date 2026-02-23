import cv2
import numpy as np
import torch

class Dashboard:
    def __init__(self, window_name="Auto Brain"):
        self.window_name = window_name
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.window_name, 1200, 600)

    def update(self, state_tensor, rotated_tensor_vol, heatmaps_vol, 
               chosen_rot_idx, u_rot, v_rot, conf, u_world, v_world):
        """
        Args:
            state_tensor: (3, H, W) Tensor (World View)
            rotated_tensor_vol: (N, 3, H, W) Tensor (All rotated inputs)
            heatmaps_vol: (N, 1, H, W) Tensor (Raw logits/probs)
            chosen_rot_idx: Int (Which rotation was picked)
            u_rot, v_rot: Ints (Coords in the rotated frame)
            conf: Float (Confidence score)
            u_world, v_world: Ints (Coords in the world frame for final verification)
        """
        
        # 1. Convert Tensors to Numpy Images
        # Helper to convert (C, H, W) -> (H, W, C) BGR for OpenCV
        def to_cv(t):
            img = t.detach().cpu().numpy().transpose(1, 2, 0)
            img = np.ascontiguousarray(img) # Fix memory layout for cv2
            return cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

        # 2. Build Top Row: Rotated Views & Heatmaps
        num_rots = rotated_tensor_vol.shape[0]
        pairs = []
        
        for i in range(num_rots):
            # Input Image
            img_cv = to_cv(rotated_tensor_vol[i])
            
            # Heatmap (Sigmoid -> ColorMap)
            hm = heatmaps_vol[i].detach().cpu().numpy().squeeze()
            hm = 1.0 / (1.0 + np.exp(-hm)) # Sigmoid
            hm_img = (hm * 255).astype(np.uint8)
            hm_color = cv2.applyColorMap(hm_img, cv2.COLORMAP_JET)
            
            # Resize heatmap to match image if needed
            if hm_color.shape[:2] != img_cv.shape[:2]:
                hm_color = cv2.resize(hm_color, (img_cv.shape[1], img_cv.shape[0]))

            # Mark Selection
            if i == chosen_rot_idx:
                # Draw selection on both
                cv2.circle(img_cv, (v_rot, u_rot), 5, (0, 0, 255), -1)
                cv2.circle(hm_color, (v_rot, u_rot), 5, (255, 255, 255), -1)
                
                # Combine & Add Border
                pair = np.hstack([img_cv, hm_color])
                cv2.rectangle(pair, (0,0), (pair.shape[1]-1, pair.shape[0]-1), (0, 255, 0), 3)
                label = f"ROT {i} (CHOSEN)"
            else:
                pair = np.hstack([img_cv, hm_color])
                label = f"ROT {i}"
                
            cv2.putText(pair, label, (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            pairs.append(pair)

        # Concatenate all rotation pairs horizontally
        top_strip = np.hstack(pairs)

        # 3. Build Bottom Row: World View
        world_cv = to_cv(state_tensor)
        
        # Draw the execution arrow in World Frame
        # Simple crosshair at target
        cv2.circle(world_cv, (v_world, u_world), 8, (0, 255, 0), 2)
        cv2.putText(world_cv, f"EXECUTE: ({u_world}, {v_world}) | Conf: {conf:.2f}", 
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        # Resize world view to match the width of the top strip (for clean stacking)
        target_width = top_strip.shape[1]
        scale = target_width / world_cv.shape[1]
        new_h = int(world_cv.shape[0] * scale)
        world_resized = cv2.resize(world_cv, (target_width, new_h))

        # 4. Final Stack & Show
        final_dashboard = np.vstack([top_strip, world_resized])
        
        # Optional: Downscale if it's too huge for the screen
        if final_dashboard.shape[1] > 1920:
             final_dashboard = cv2.resize(final_dashboard, (0,0), fx=0.7, fy=0.7)

        cv2.imshow(self.window_name, final_dashboard)
        cv2.waitKey(1)