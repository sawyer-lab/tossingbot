import torch

def predict_action(model, rotated_batch):
    """
    Runs inference on a batch of rotated images and finds the global argmax.
    """
    model.eval()
    with torch.no_grad():
        # 1. Forward Pass (Batch of N rotations)
        # rotated_batch should already be on device from the utils step
        logits = model(rotated_batch)
        probs = torch.sigmoid(logits)
        
        # 2. Find Global Max
        # Flatten: (N, 1, H, W) -> (N * H * W)
        flat_idx = torch.argmax(probs).item()
        
        # 3. Unravel Index
        N, _, H, W = probs.shape
        stride = H * W
        
        rot_idx = flat_idx // stride
        pixel_idx = flat_idx % stride
        
        u = pixel_idx // W
        v = pixel_idx % W
        
        best_conf = probs.view(-1)[flat_idx].item()
        
        return rot_idx, u, v, best_conf, probs


def train_step_per(model, optimizer, loss_fn, batch_data, device):
    """
    Training step for Prioritized Experience Replay.
    Assumes inputs are already on device (or easily movable) and loss_fn has reduction='none'.
    """
    model.train()
    
    # 1. Unpack Data
    images, targets_u, targets_v, rewards, is_weights = batch_data
    
    images = images.to(device)
    rewards = rewards.to(device)
    is_weights = torch.as_tensor(is_weights, dtype=torch.float32, device=device).unsqueeze(1)
    
    # Ensure indices are tensors on device for advanced indexing
    targets_u = torch.as_tensor(targets_u, device=device)
    targets_v = torch.as_tensor(targets_v, device=device)

    optimizer.zero_grad()
    
    # 2. Forward Pass
    logits = model(images)
    
    # 3. Vectorized Gathering
    # Select logits only at the (u,v) where we acted
    batch_idx = torch.arange(len(rewards), device=device)
    pred_vals = logits[batch_idx, 0, targets_u, targets_v].unsqueeze(1)

    # 4. Calculate Loss
    # Assumes loss_fn is initialized with reduction='none'
    raw_loss = loss_fn(pred_vals, rewards)
    
    # 5. Apply Importance Sampling & Backprop
    loss_to_optimize = (raw_loss * is_weights).mean()
    
    loss_to_optimize.backward()
    optimizer.step()
    
    # 6. Return weighted loss (for logs) and raw errors (for buffer update)
    errors = raw_loss.detach().cpu().numpy().flatten()
    
    return loss_to_optimize.item(), errors