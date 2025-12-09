import torch
import torch.nn as nn
import torch.nn.functional as F

class ResidualBlock(nn.Module):
    """
    Standard Residual Block [14]: Conv -> BN -> ReLU -> Conv -> BN -> Add -> ReLU
    """
    def __init__(self, in_channels, out_channels):
        super(ResidualBlock, self).__init__()
        
        # If channels change, we need to adapt the identity line (shortcut)
        self.shortcut = nn.Sequential()
        if in_channels != out_channels:
            self.shortcut = nn.Sequential(
                nn.Conv2d(in_channels, out_channels, kernel_size=1, bias=False),
                nn.BatchNorm2d(out_channels)
            )

        self.conv1 = nn.Conv2d(in_channels, out_channels, kernel_size=3, padding=1, bias=False)
        self.bn1 = nn.BatchNorm2d(out_channels)
        
        self.conv2 = nn.Conv2d(out_channels, out_channels, kernel_size=3, padding=1, bias=False)
        self.bn2 = nn.BatchNorm2d(out_channels)

    def forward(self, x):
        residual = self.shortcut(x)
        
        out = F.relu(self.bn1(self.conv1(x)))
        out = self.bn2(self.conv2(out))
        
        out += residual
        out = F.relu(out)
        return out

class TossingBot(nn.Module):
    def __init__(self, input_channels=3):
        super(TossingBot, self).__init__()
        
        # ======================================================================
        # 1. PERCEPTION MODULE (Encoder)
        # Architecture: C(3,64) -> MP -> RB(128) -> MP -> RB(256) -> RB(512)
        # ======================================================================
        
        # C(3, 64) - Note: Kernel size 3x3 implies padding=1 to keep size
        self.perc_conv1 = nn.Conv2d(input_channels, 64, kernel_size=3, padding=1, bias=False)
        self.perc_bn1 = nn.BatchNorm2d(64)
        
        # MP occurs in forward()
        
        self.perc_rb1 = ResidualBlock(64, 128)   # RB(128)
        # MP occurs in forward()
        self.perc_rb2 = ResidualBlock(128, 256)  # RB(256)
        self.perc_rb3 = ResidualBlock(256, 512)  # RB(512)
        
        # Output here is 'mu' (The feature embedding)

        # ======================================================================
        # 2. GRASPING MODULE (Decoder)
        # Architecture: RB(256) -> RB(128) -> UP -> RB(64) -> UP -> C(1, 1)
        # ======================================================================
        
        self.grasp_rb1 = ResidualBlock(512, 256) # RB(256)
        self.grasp_rb2 = ResidualBlock(256, 128) # RB(128)
        
        # UP occurs in forward()
        
        self.grasp_rb3 = ResidualBlock(128, 64)  # RB(64)
        
        # UP occurs in forward()
        
        # Final Convolution: C(1, 1) -> 1x1 Kernel, 1 Output Channel (Probability)
        # Note: Bias is True here because this is the final logit
        self.grasp_conv_final = nn.Conv2d(64, 1, kernel_size=1, bias=True)

    def forward(self, x):
        """
        Args:
            x: (Batch, Channels, H, W)
        Returns:
            logits: (Batch, 1, H, W) -> Not Sigmoided yet!
        """
        # Save original size for exact reconstruction
        # This handles cases where H or W aren't perfect powers of 2
        orig_size = x.shape[2:] 
        
        # --- PERCEPTION ---
        # 1. C(3,64)
        x = F.relu(self.perc_bn1(self.perc_conv1(x)))
        
        # 2. Max Pool (Stride 2)
        x = F.max_pool2d(x, kernel_size=3, stride=2, padding=1)
        
        # 3. RB(128)
        x = self.perc_rb1(x)
        
        # 4. Max Pool (Stride 2)
        x = F.max_pool2d(x, kernel_size=3, stride=2, padding=1)
        
        # 5. RB(256) -> RB(512)
        x = self.perc_rb2(x)
        mu = self.perc_rb3(x) # This is the bottleneck feature map
        
        # --- GRASPING ---
        g = self.grasp_rb1(mu)
        g = self.grasp_rb2(g)
        
        # Upsample 1
        g = F.interpolate(g, scale_factor=2, mode='bilinear', align_corners=True)
        
        g = self.grasp_rb3(g)
        
        # Upsample 2 (Force exact match to input size)
        g = F.interpolate(g, size=orig_size, mode='bilinear', align_corners=True)
        
        # Final Prediction
        g = self.grasp_conv_final(g)
        
        return g
    
