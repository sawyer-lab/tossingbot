#!/usr/bin/env python3.8
import rospy
import torch
from tossingbot.learning.base_node import BaseLearningNode

class AutoTrainer(BaseLearningNode):
    def __init__(self):
        super().__init__("auto_trainer", training_enabled=True)
        self.TRAIN_INTERVAL = 1
        self.BURST_SIZE = 5
        self.BATCH_SIZE = 16
        self.LEARNING_RATE = 1e-4

        rospy.loginfo("--- SELF-SUPERVISED MODE: Auto Collecting & Training ---")

    def run(self):
        while not rospy.is_shutdown():
            state, heatmap = self.get_heatmap()
            if state is None: 
                rospy.sleep(0.1); continue

            # 1. Select Action (Greedy)
            flat_idx = torch.argmax(heatmap).item()
            u = flat_idx // heatmap.shape[1]
            v = flat_idx % heatmap.shape[1]
            
            conf = torch.sigmoid(heatmap[u, v]).item()

            # 2. Execute
            rospy.loginfo(f"Auto Action: ({u}, {v}) | Conf: {conf:.2f}")
            self.execute_and_store(state, u, v)
            
            # 3. Visualize
            self.visualize(state, heatmap, u, v, extra_text=f"Conf: {conf:.2f}")

            # 4. Train Logic (Burst)
            if self.step_count % self.TRAIN_INTERVAL == 0:
                self.train_burst(iterations=self.BURST_SIZE)
                self.save_weights()

if __name__ == "__main__":
    AutoTrainer().run()