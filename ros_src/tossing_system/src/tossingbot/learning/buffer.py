import random
import torch

class ReplayBuffer:
    def __init__(self, capacity=10000):
        self.capacity = capacity
        self.memory = []
        self.position = 0

    def push(self, state, u, v, reward):
        """
        state: torch.Tensor (3, H, W)
        u, v: integers (pixel coords)
        reward: float
        """
        # Store on CPU to save VRAM
        # We store (state, u, v, reward)
        if state is not None:
            state_cpu = state.cpu()
        else:
            return

        transition = (state_cpu, u, v, float(reward))
        
        if len(self.memory) < self.capacity:
            self.memory.append(transition)
        else:
            self.memory[self.position] = transition
            self.position = (self.position + 1) % self.capacity

    def sample(self, batch_size):
        # Prevent sampling more than we have
        count = min(len(self.memory), batch_size)
        return random.sample(self.memory, count)

    def __len__(self):
        return len(self.memory)