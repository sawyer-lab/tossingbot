import random
import torch
import numpy as np

class ReplayBuffer:
    def __init__(self, capacity=2000):
        self.capacity = capacity
        
        # Separate lists for balancing
        self.successes = []
        self.failures = []
        
        # Pointers for ring-buffer logic (optional, simpler to just append/pop for lists)
        self.pos_s = 0
        self.pos_f = 0

    def push_rotational(self, state, u, v, rot, reward):
        """
        Stores experience in the correct bucket.
        """
        if state is None: return
        state_cpu = state.cpu()

        # Transition tuple
        transition = (state_cpu, u, v, rot, float(reward))
        
        # Check Reward to decide bucket
        # We assume Reward > 0.5 is Success
        if reward > 0.5:
            if len(self.successes) < self.capacity:
                self.successes.append(transition)
            else:
                # Overwrite old successes (Ring buffer style)
                self.successes[self.pos_s] = transition
                self.pos_s = (self.pos_s + 1) % self.capacity
        else:
            if len(self.failures) < self.capacity:
                self.failures.append(transition)
            else:
                self.failures[self.pos_f] = transition
                self.pos_f = (self.pos_f + 1) % self.capacity

    def sample(self, batch_size):
        """
        Returns a balanced batch (50% success, 50% fail).
        If we don't have enough successes, we duplicate them.
        """
        n_success = len(self.successes)
        n_fail = len(self.failures)
        
        if n_fail == 0: return [] # Can't train without data
        
        # Strategy: Try to get half/half
        want_s = batch_size // 2
        want_f = batch_size - want_s
        
        # 1. Sample Failures (Easy)
        batch = random.sample(self.failures, min(n_fail, want_f))
        
        # 2. Sample Successes (Harder - might be 0)
        if n_success > 0:
            # If we have successes, sample them (with replacement if needed)
            if n_success < want_s:
                # We have fewer successes than we want. Repeat them!
                # This is CRITICAL. Show that one banana grasp 5 times in a row.
                s_samples = [random.choice(self.successes) for _ in range(want_s)]
            else:
                s_samples = random.sample(self.successes, want_s)
            
            batch += s_samples
        
        # If we still don't have enough (e.g. 0 successes), fill with failures
        while len(batch) < batch_size and n_fail > 0:
            batch.append(random.choice(self.failures))
            
        random.shuffle(batch)
        return batch

    def __len__(self):
        return len(self.successes) + len(self.failures)