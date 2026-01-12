import numpy as np
import torch

class RankBasedReplayBuffer:
    def __init__(self, capacity, alpha=1.0):
        """
        Rank-Based Prioritized Experience Replay.
        
        Args:
            capacity (int): Max number of transitions to store.
            alpha (float): 0.0 = Uniform Random (No priority).
                           1.0 = Fully Rank-Based (Heavy bias towards hard tasks).
                           (Updated to 1.0 to match paper implementation style)
        """
        self.capacity = capacity
        self.alpha = alpha
        self.buffer = []
        self.priorities = []
        self.position = 0

    def push(self, state, u, v, rot, reward):
        """Saves a transition."""
        # New samples get max priority so they are guaranteed to be sampled at least once
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
        
        # --- 1. RANKING (The core logic) ---
        # Sort indices by priority (Descending: High Error -> Rank 1)
        sorted_indices = np.argsort(self.priorities)[::-1]
        
        # --- 2. CALCULATE PROBS ---
        # P(i) = 1 / rank^alpha
        ranks = np.arange(1, N + 1)
        probs = (1.0 / ranks) ** self.alpha
        probs /= probs.sum() # Normalize
        
        # --- 3. SAMPLE ---
        chosen_ranks = np.random.choice(N, batch_size, p=probs)
        batch_indices = sorted_indices[chosen_ranks]
        batch = [self.buffer[idx] for idx in batch_indices]
        
        # --- 4. DEBUG PRINTS (Distribution Check) ---
        self._print_stats(batch)

        return batch, batch_indices

    def update_priorities(self, indices, errors):
        """Updates the priorities (errors) of the sampled batch."""
        for idx, err in zip(indices, errors):
            # Convert to float to avoid numpy array nesting issues
            priority = abs(float(err)) + 1e-5
            self.priorities[idx] = priority

    def _print_stats(self, batch):
        """
        Helper to visualize if positive examples are actually being sampled.
        """
        # Count Positives (Reward > 0.5) in the Current Batch
        batch_rewards = [b[4] for b in batch]
        n_pos_batch = sum(r > 0.5 for r in batch_rewards)
        
        # Count Positives in the Entire Buffer (Can be slow if buffer is huge, fine for <10k)
        # We only do this check occasionally or if buffer is small
        if len(self.buffer) > 0:
            all_rewards = [b[4] for b in self.buffer]
            n_pos_total = sum(r > 0.5 for r in all_rewards)
            total = len(self.buffer)
            
            # Get Top 5 Priorities to verify "Surprise" sorting
            top_prios = sorted(self.priorities, reverse=True)[:5]
            top_prios_str = [f"{p:.4f}" for p in top_prios]

            print(f"\n--- BUFFER DEBUG ---")
            print(f"Global Buffer: {n_pos_total} Pos / {total} Total ({n_pos_total/total*100:.1f}%)")
            print(f"Current Batch: {n_pos_batch} Pos / {len(batch)} Total")
            print(f"Top Priorities: {top_prios_str}")
            if n_pos_batch > 0:
                print(f"SUCCESS SAMPLED! The prioritization is working.")
            else:
                print(f"No success in this batch (Normal if global success is < 1%)")

    def __len__(self):
        return len(self.buffer)