import torch
import torch.nn as nn
import torch.optim as optim

# 定义一个简单的神经网络模型
class SamplingPolicy(nn.Module):
    def __init__(self):
        super(SamplingPolicy, self).__init__()
        self.fc1 = nn.Linear(4, 64)  # 输入: [current_x, current_y, goal_x, goal_y]
        self.fc2 = nn.Linear(64, 64)
        self.fc3 = nn.Linear(64, 2)  # 输出: [predicted_x, predicted_y]

    def forward(self, x):
        x = torch.relu(self.fc1(x))
        x = torch.relu(self.fc2(x))
        x = self.fc3(x)
        return x

# 创建模型实例
model = SamplingPolicy()

# 定义损失函数和优化器
criterion = nn.MSELoss()
optimizer = optim.Adam(model.parameters(), lr=0.001)

# 生成一些随机训练数据
# 输入: [current_x, current_y, goal_x, goal_y]
# 输出: [predicted_x, predicted_y]
inputs = torch.randn(100, 4)  # 100个样本
targets = torch.randn(100, 2)  # 100个目标

# 训练模型
for epoch in range(100):
    optimizer.zero_grad()
    outputs = model(inputs)
    loss = criterion(outputs, targets)
    loss.backward()
    optimizer.step()
    if epoch % 10 == 0:
        print(f"Epoch {epoch}, Loss: {loss.item()}")

# 保存模型
torch.save(model.state_dict(), 'sampling_policy.pth')
print("Model saved to sampling_policy.pth")