#!/usr/bin/env python3.8
import rospy
import cv2
import numpy as np
from tossingbot.learning.base_node import BaseLearningNode

class ManualTrainer(BaseLearningNode):
    def __init__(self):
        super().__init__("manual_trainer", training_enabled=True)

        self.BATCH_SIZE = 8
        self.LEARNING_RATE = 2e-4 
        self.BURST_SIZE = 10
        self.TRAIN_INTERVAL = 1
        
        self.pending_click = None
        cv2.setMouseCallback("TossingBot View", self._mouse_cb)
        rospy.loginfo("--- MANUAL MODE: Click to Grasp & Train ---")

    def _mouse_cb(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.pending_click = (x, y)

    def run(self):
        while not rospy.is_shutdown():
            state, heatmap = self.get_heatmap()
            if state is None: 
                rospy.sleep(0.1); continue

            # If user clicked...
            if self.pending_click:
                win_x, win_y = self.pending_click
                self.pending_click = None # Reset
                
                # Unscale Window -> Grid
                scale = 3.0 # Defined in visualize
                v = int(np.clip(win_x / scale, 0, heatmap.shape[1]-1))
                u = int(np.clip(win_y / scale, 0, heatmap.shape[0]-1))
                
                # Check side (if clicked on heatmap)
                if v >= state.shape[2]: v -= state.shape[2]

                rospy.loginfo(f"Manual Action: ({u}, {v})")
                
                # 1. Execute
                label = self.execute_and_store(state, u, v)
                
                # 2. Train Immediately (1-shot learning)
                # Since manual data is precious, we train heavily on it immediately
                self.train_burst(iterations=self.BURST_SIZE)
                
                # 3. Save
                if self.step_count % 10 == 0: self.save_weights()

            # Visualize loop
            # Just show center if no action pending
            self.visualize(state, heatmap, 0, 0, extra_text="Waiting for Click...")

if __name__ == "__main__":
    ManualTrainer().run()