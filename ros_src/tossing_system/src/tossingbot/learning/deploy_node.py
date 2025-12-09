#!/usr/bin/env python3.8
import rospy
import torch
from tossingbot.learning.base_node import BaseLearningNode

class DeployNode(BaseLearningNode):
    def __init__(self):
        # Disable Training -> Eval Mode, No Buffer
        super().__init__("deploy_node", training_enabled=False)
        rospy.loginfo("--- DEPLOY MODE: Pure Inference ---")

    def run(self):
        while not rospy.is_shutdown():
            state, heatmap = self.get_heatmap()
            if state is None: 
                rospy.sleep(0.1); continue

            # 1. Select Action
            flat_idx = torch.argmax(heatmap).item()
            u = flat_idx // heatmap.shape[1]
            v = flat_idx % heatmap.shape[1]
            conf = torch.sigmoid(heatmap[u, v]).item()

            # 2. Execute (No storage)
            rospy.loginfo(f"Deploy Action: ({u}, {v}) | Conf: {conf:.2f}")
            
            # Note: execute_and_store handles reset, but won't buffer because training_enabled=False
            self.execute_and_store(state, u, v)
            
            # 3. Visualize
            self.visualize(state, heatmap, u, v, extra_text=f"DEPLOYING | Conf: {conf:.2f}")

if __name__ == "__main__":
    DeployNode().run()