import torch
import torch.nn as nn
import torch.nn.functional as F

class ResidualBlock(nn.Module):
    """
    Standard Residual Block: Conv -> BN -> ReLU -> Conv -> BN -> Add -> ReLU
    """
    def __init__(self, in_channels, out_channels):
        super(ResidualBlock, self).__init__()
        
        # Shortcut connection
        self.shortcut = nn.Sequential()
        if in_channels != out_channels:
            self.shortcut = nn.Sequential(
                nn.Conv2d(in_channels, out_channels, kernel_size=1, bias=False),
                nn.BatchNorm2d(out_channels)
            )

        # Main path
        self.conv1 = nn.Conv2d(in_channels, out_channels, kernel_size=3, padding=1, bias=False)
        self.bn1 = nn.BatchNorm2d(out_channels)
        self.conv2 = nn.Conv2d(out_channels, out_channels, kernel_size=3, padding=1, bias=False)
        self.bn2 = nn.BatchNorm2d(out_channels)

    def forward(self, x):
        residual = self.shortcut(x)
        
        out = F.relu(self.bn1(self.conv1(x)))
        out = self.bn2(self.conv2(out))
        
        # Add shortcut
        out += residual
        out = F.relu(out)
        return out

class PerceptionModule(nn.Module):
    """
    Encoder: C(4,64) -> MP -> RB(128) -> MP -> RB(256) -> RB(512)
    """
    def __init__(self, input_channels=4):
        super(PerceptionModule, self).__init__()
        
        # Initial Conv (Accepts 4 channels: RGBD)
        self.conv_in = nn.Conv2d(input_channels, 64, kernel_size=3, padding=1, bias=False)
        self.bn_in = nn.BatchNorm2d(64)
        
        self.rb1 = ResidualBlock(64, 128)    
        self.rb2 = ResidualBlock(128, 256)   
        self.rb3 = ResidualBlock(256, 512)   

    def forward(self, x):
        x = F.relu(self.bn_in(self.conv_in(x)))
        x = F.max_pool2d(x, kernel_size=3, stride=2, padding=1)
        
        x = self.rb1(x)
        x = F.max_pool2d(x, kernel_size=3, stride=2, padding=1)
        
        x = self.rb2(x)
        mu = self.rb3(x) 
        
        return mu
    
class GraspingModule(nn.Module):
    """
    Decoder: RB(512->256) -> RB(256->128) -> UP -> RB(128->64) -> UP -> C(64, 1)
    """
    def __init__(self):
        super(GraspingModule, self).__init__()
        
        self.rb1 = ResidualBlock(512, 256)
        self.rb2 = ResidualBlock(256, 128)
        self.rb3 = ResidualBlock(128, 64)
        self.conv_final = nn.Conv2d(64, 1, kernel_size=1, bias=True)

    def forward(self, mu, target_size):
        g = self.rb1(mu)
        g = self.rb2(g)
        g = F.interpolate(g, scale_factor=2, mode='bilinear', align_corners=True)
        g = self.rb3(g)
        g = F.interpolate(g, size=target_size, mode='bilinear', align_corners=True)
        logits = self.conv_final(g)
        
        return logits
    
class TossingBot_Modular(nn.Module):
    """
    Top-level model composed of a Perception (Encoder) and Grasping (Decoder) module.
    """
    def __init__(self, input_channels=4):
        super(TossingBot_Modular, self).__init__()
        
        self.perception_module = PerceptionModule(input_channels)
        self.grasping_module = GraspingModule()
        
        # Apply Random Initialization
        self.apply(self._init_weights)
        
    def _init_weights(self, m):
        """
        Initializes BOTH weights and biases randomly to ensure static noise output.
        """
        if isinstance(m, (nn.Conv2d, nn.Linear)):
            # Weights: Random Kaiming Normal
            nn.init.kaiming_normal_(m.weight, mode='fan_out', nonlinearity='relu')
            
            # Biases: Random Uniform
            if m.bias is not None:
                nn.init.uniform_(m.bias, -0.1, 0.1)
                
        elif isinstance(m, nn.BatchNorm2d):
            # BN Weights: Random around 1
            nn.init.uniform_(m.weight, 0.9, 1.1)
            # BN Bias: Random around 0
            nn.init.uniform_(m.bias, -0.1, 0.1)

    def forward(self, x):
        """
        Args:
            x: (Batch, Channels, H, W)
        Returns:
            logits: (Batch, 1, H, W)
        """
        orig_size = x.shape[2:] 
        mu = self.perception_module(x)
        logits = self.grasping_module(mu, orig_size)
        
        return logits