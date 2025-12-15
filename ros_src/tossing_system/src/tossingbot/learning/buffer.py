import numpy as np
import random

class SumTree:
    """
    A binary tree data structure where the parent's value is the sum of its children.
    This allows for O(log n) sampling based on priority.
    """
    def __init__(self, capacity):
        self.capacity = capacity
        # Tree size is 2*capacity - 1
        # Indices 0 to capacity-2 are branches, capacity-1 to end are leaves (data)
        self.tree = np.zeros(2 * capacity - 1)
        self.data = np.zeros(capacity, dtype=object)
        self.write_idx = 0
        self.n_entries = 0

    def add(self, priority, data):
        """Add data with a specific priority."""
        idx = self.write_idx + self.capacity - 1

        self.data[self.write_idx] = data
        self.update(idx, priority)

        self.write_idx += 1
        if self.write_idx >= self.capacity:
            self.write_idx = 0
            
        if self.n_entries < self.capacity:
            self.n_entries += 1

    def update(self, idx, priority):
        """Update the priority of a leaf and propagate the change up the tree."""
        change = priority - self.tree[idx]
        self.tree[idx] = priority
        
        # Propagate change up the tree
        while idx != 0:
            idx = (idx - 1) // 2
            self.tree[idx] += change

    def get_total_priority(self):
        return self.tree[0]

    def get(self, s):
        """
        Retrieve a leaf node based on a cumulative value 's'.
        Returns: (tree_index, priority, data_object)
        """
        parent_idx = 0
        
        while True:
            left_child_idx = 2 * parent_idx + 1
            right_child_idx = left_child_idx + 1
            
            # If we reach the bottom (leaf nodes), break
            if left_child_idx >= len(self.tree):
                leaf_idx = parent_idx
                break
            
            if s <= self.tree[left_child_idx]:
                parent_idx = left_child_idx
            else:
                s -= self.tree[left_child_idx]
                parent_idx = right_child_idx
        
        data_idx = leaf_idx - (self.capacity - 1)
        return leaf_idx, self.tree[leaf_idx], self.data[data_idx]

class PrioritizedReplayBuffer:
    """
    In-memory Prioritized Experience Replay (PER).
    Stores transitions and samples them based on their TD-error (loss).
    """
    def __init__(self, capacity=10000, alpha=0.6, beta_start=0.4, beta_frames=1000):
        """
        Args:
            capacity: Max number of transitions.
            alpha: How much prioritization is used (0 = uniform, 1 = full priority).
            beta_start: Importance sampling weight (0 = no correction, 1 = full correction).
            beta_frames: Number of steps to anneal beta from beta_start to 1.0.
        """
        self.tree = SumTree(capacity)
        self.alpha = alpha
        self.beta_start = beta_start
        self.beta_frames = beta_frames
        self.frame = 0
        self.epsilon = 1e-5 # Small constant to prevent zero priority

    def push(self, state, u, v, rot_idx, reward):
        """
        Add a new experience.
        New experiences are usually added with max priority to ensure they are trained on at least once.
        """
        # Find current max priority
        # If tree is empty, use 1.0, else max of existing priorities
        max_p = np.max(self.tree.tree[-self.tree.capacity:])
        if max_p == 0: max_p = 1.0
            
        data = (state, u, v, rot_idx, reward)
        self.tree.add(max_p, data)

    def sample(self, batch_size):
        """
        Sample a batch of transitions.
        Returns:
            batch: List of (state, u, v, rot_idx, reward)
            indices: List of tree indices (needed to update priorities later)
            weights: Importance sampling weights for loss correction
        """
        batch = []
        indices = []
        weights = []
        segment = self.tree.get_total_priority() / batch_size
        
        # Calculate current beta
        self.frame += 1
        beta = min(1.0, self.beta_start + self.frame * (1.0 - self.beta_start) / self.beta_frames)
        
        # Calculate max weight for normalization
        # p_min = min_prob / total_priority
        # min_prob is essentially the min non-zero value in the tree
        # For efficiency we can just take the min of the current batch or estimate
        # Ideally: p_min = np.min(self.tree.tree[-self.tree.n_entries:]) / self.tree.get_total_priority()
        # But calculating min of tree is O(N). Simplification: assume 1/N
        p_min = 1.0 / max(1, self.tree.n_entries) 
        max_weight = (p_min * self.tree.n_entries) ** (-beta)

        for i in range(batch_size):
            a = segment * i
            b = segment * (i + 1)
            s = random.uniform(a, b)
            
            idx, priority, data = self.tree.get(s)
            
            # Probability of sampling this item
            p = priority / self.tree.get_total_priority()
            
            # Importance Sampling Weight: w = (N * P(i)) ^ -beta
            # We normalize by max_weight for stability
            if p == 0: p = 1e-5 # prevent div by zero
            weight = ((self.tree.n_entries * p) ** (-beta)) / max_weight
            
            batch.append(data)
            indices.append(idx)
            weights.append(weight)

        return batch, indices, np.array(weights, dtype=np.float32)

    def update_priorities(self, indices, errors):
        """
        Update priorities after a training step.
        errors: The loss/TD-error for each sample in the batch.
        """
        for idx, error in zip(indices, errors):
            # Priority = (|error| + epsilon) ^ alpha
            p = (np.abs(error) + self.epsilon) ** self.alpha
            self.tree.update(idx, p)
            
    def __len__(self):
        return self.tree.n_entries