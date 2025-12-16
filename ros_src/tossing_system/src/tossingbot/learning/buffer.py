import numpy as np
import torch

class RankBasedReplayBuffer:
    def __init__(self, capacity, alpha=0.5):
        """
        Rank-Based Prioritized Experience Replay.
        
        Args:
            capacity (int): Max number of transitions to store.
            alpha (float): How much prioritization to use.
                           0.0 = Uniform Random (No priority).
                           1.0 = Fully greedy (Only pick hard tasks).
                           Paper recommends ~0.5 for robust training.
        """
        self.capacity = capacity
        self.alpha = alpha
        self.buffer = []
        self.priorities = []
        self.position = 0

    def push(self, state, u, v, rot, reward):
        """Saves a transition."""
        max_prio = max(self.priorities) if self.priorities else 1.0
        
        transition = (state, u, v, rot, reward)
        
        if len(self.buffer) < self.capacity:
            self.buffer.append(transition)
            self.priorities.append(max_prio)
        else:
            self.buffer[self.position] = transition
            self.priorities[self.position] = max_prio
            self.position = (self.position + 1) % self.capacity

    def sample(self, batch_size):
        N = len(self.buffer)
        if N == 0: return [], [], []
        
        # 1. Sort indices by priority (Descending: High Error -> Rank 1)
        # argsort gives ascending, so we flip it [::-1]
        sorted_indices = np.argsort(self.priorities)[::-1]
        
        # 2. Calculate Probabilities based on RANK (1 / rank^alpha)
        # Ranks are 1, 2, ..., N
        ranks = np.arange(1, N + 1)
        probs = (1.0 / ranks) ** self.alpha
        probs /= probs.sum() # Normalize to sum to 1
        
        # 3. Sample indices based on these probabilities
        # We sample from 0..N-1, but using the sorted_indices map
        chosen_ranks = np.random.choice(N, batch_size, p=probs)
        batch_indices = sorted_indices[chosen_ranks]
        
        batch = [self.buffer[idx] for idx in batch_indices]
        
        # Return batch, indices (for updating later), and NO WEIGHTS (Paper doesn't use IS)
        return batch, batch_indices

    def update_priorities(self, indices, errors):
        """Updates the priorities (errors) of the sampled batch."""
        for idx, err in zip(indices, errors):
            # FIX: Ensure we extract the value as a standard python float
            # .item() handles both 0-d numpy arrays and single-element tensors
            priority = abs(float(err)) + 1e-5
            self.priorities[idx] = priority

    def __len__(self):
        return len(self.buffer)