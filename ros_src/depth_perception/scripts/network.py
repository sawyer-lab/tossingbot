import torch
import torch.nn as nn
import torch.nn.functional as F

class ResidualBlock(nn.Module):
    """
    Standard ResNet Block:
    Input -> Conv -> BN -> ReLU -> Conv -> BN -> (+ Input) -> ReLU
    Includes a shortcut (1x1 conv) if channel dimensions change.
    """
    def __init__(self, in_c, out_c):
        super().__init__()
        self.conv1 = nn.Conv2d(in_c, out_c, kernel_size=3, padding=1)
        self.bn1 = nn.BatchNorm2d(out_c)
        self.conv2 = nn.Conv2d(out_c, out_c, kernel_size=3, padding=1)
        self.bn2 = nn.BatchNorm2d(out_c)

        # If input and output channels differ, we need to project the identity connection
        self.shortcut = nn.Sequential()
        if in_c != out_c:
            self.shortcut = nn.Sequential(
                nn.Conv2d(in_c, out_c, kernel_size=1),
                nn.BatchNorm2d(out_c)
            )

    def forward(self, x):
        out = F.relu(self.bn1(self.conv1(x)))
        out = self.bn2(self.conv2(out))
        out += self.shortcut(x)
        return F.relu(out)

class PerceptionNet(nn.Module):
    """
    Specs: C(3,64)-MP-RB(128)-MP-RB(256)-RB(512)
    """
    def __init__(self):
        super().__init__()
        # C(3, 64)
        self.init_conv = nn.Sequential(
            nn.Conv2d(3, 64, kernel_size=3, padding=1),
            nn.BatchNorm2d(64),
            nn.ReLU()
        )
        # MP
        self.pool = nn.MaxPool2d(kernel_size=2, stride=2)
        
        # RB(128)
        self.rb1 = ResidualBlock(64, 128)
        
        # RB(256)
        self.rb2 = ResidualBlock(128, 256)
        
        # RB(512)
        self.rb3 = ResidualBlock(256, 512)

    def forward(self, x):
        # x: (Batch, 3, H, W)
        x = self.init_conv(x)   # -> (64, H, W)
        x = self.pool(x)        # -> (64, H/2, W/2)
        x = self.rb1(x)         # -> (128, H/2, W/2)
        x = self.pool(x)        # -> (128, H/4, W/4)
        x = self.rb2(x)         # -> (256, H/4, W/4)
        x = self.rb3(x)         # -> (512, H/4, W/4)
        return x # These are the "pixel parameters" (features)

class GraspingNet(nn.Module):
    """
    Specs: RB(256)-RB(128)-UP-RB(64)-UP-C(1,2)
    """
    def __init__(self):
        super().__init__()
        
        # Input comes from Perception (512 channels)
        # The first RB needs to bring 512 -> 256
        self.rb1 = ResidualBlock(512, 256)
        
        # RB(128)
        self.rb2 = ResidualBlock(256, 128)
        
        # UP (Bilinear Upsample)
        self.up = nn.Upsample(scale_factor=2, mode='bilinear', align_corners=True)
        
        # RB(64)
        self.rb3 = ResidualBlock(128, 64)
        
        # Final Conv C(1, 2) -> Kernel 1, 2 Output Channels
        self.final_conv = nn.Conv2d(64, 2, kernel_size=1)

    def forward(self, x):
        # x: (Batch, 512, H/4, W/4)
        x = self.rb1(x)         # -> (256, H/4, W/4)
        x = self.rb2(x)         # -> (128, H/4, W/4)
        x = self.up(x)          # -> (128, H/2, W/2)
        x = self.rb3(x)         # -> (64, H/2, W/2)
        x = self.up(x)          # -> (64, H, W)
        x = self.final_conv(x)  # -> (2, H, W)
        return x

class TossingBot(nn.Module):
    """Wrapper that connects Perception -> Grasping"""
    def __init__(self):
        super().__init__()
        self.perception = PerceptionNet()
        self.grasping = GraspingNet()

    def forward(self, x):
        features = self.perception(x)
        output = self.grasping(features)
        return output