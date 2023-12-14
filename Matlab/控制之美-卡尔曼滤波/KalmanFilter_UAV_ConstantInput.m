clear all
close all
clc

%% 定义连续系统：无人机高度控制
% 《控制之美 卷2》P41
% 构建系统矩阵A
A = [0 1 0; 
     0 0 1;
     0 0 0];
% 计算A矩阵维度：取A的行数
n = size (A,1);
% 构建输入矩阵B
B = [0; 
     1; 
     0];
% y = C*x + D*u
C = [1 0 0;
     0 1 0];
D = 0;
% 计算输入矩阵维度：取B的列数
p = size(B,2);
% 定义测量矩阵H_m
H_m = [1 0 0; 
       0 1 0; 
       0 0 1];
% 重力加速度常数
g = 10;

%% 连续系统离散化
% 离散时间步长
Ts = 0.1;
% 连续系统转离散系统 - continuous to discrete
sys_d = c2d(ss(A,B,C,D),Ts);
% 提取离散系统A矩阵
A = sys_d.a;
% 提取离散系统B矩阵
B = sys_d.b;

%% 标定卡尔曼滤波器参数
% 定义过程噪声协方差矩阵
Q_c = [0.01   0    0; 
       0      0.01 0;
       0      0    0];
% 定义测量噪声协方差矩阵
R_c = [1 0 0;
       0 1 0;
       0 0 0];

%% 给定系统初始状态
% [初始高度 初始速度 g]
x0 = [0; 1 ; -10];
% 初始化状态赋值
x = x0;
% 系统输入初始化
u0 = g;
% 初始输入赋值
u = u0;
% 初始化后验估计
x_hat0 = [0; 1; -10];
% 初始化后验估计赋值
x_hat = x_hat0;
% 初始化后验估计误差协方差矩阵
P0 = [1 0 0;
      0 1 0;
      0 0 0];
% 初始后验估计误差协方差矩阵赋值
P = P0;

%% 初始化一些结果矩阵用于存储结果
% 定义系统运行步数
k_steps = 100;
% 定义x_history零矩阵，用于储存系统状态结果，维度n x k_steps
x_history = zeros(n,k_steps);
% 定义u_history零矩阵，用于储存系统输入结果，维度p x k_steps
u_history = zeros(p,k_steps);
% 定义x_hat_history零矩阵，用于储存后验估计结果，维度n x k_steps
x_hat_history = zeros(n,k_steps);
% 定义x_hat_minus_history零矩阵，用于储存先验估计结果，维度n x k_steps
x_hat_minus_history = zeros(n,k_steps);
% 定义z_historyy零矩阵，用于测量结果，维度n x k_steps
z_history = zeros(n,k_steps);

%% 定义噪声（直接读取噪声文件）
% 定义过程噪声矩阵w，维度n x k_steps
w = zeros (n,k_steps);
% 定义测量噪声矩阵V，维度n x k_steps
v = zeros (n,k_steps);
% 从文件NoiseData.csv中读取数据
% 数据来自于系统随机生成，保存为文件可以方便进行多组实验之间的对比
noise_data = readmatrix('NoiseData.csv');
w = noise_data(2:4, :);
v = noise_data(6:8, :);
% %%%%%%%%%%%%%生成过程与测量噪声%%%%%%%%%%%%%%%%%%
% % 使用以下代码生成随机噪声，之后保存在NoiseData.csv中，方便下次读取。%%%%%%

%% 定义真实噪声
% % 定义真实的过程噪声协方差矩阵
% Q_ca = [0.05 0; 0 0.05];
% % 定义真实的测量噪声协方差矩阵
% R_ca = [1 0; 0 1];
% % 随机生成过程噪声
% w(1:2,:) = chol(Q_ca)* randn(2,k_steps);
% % 随机生成测量噪声
% v(1:2,:) = chol(R_ca)* randn(2,k_steps);

%% 仿真开始，建立for循环
for k = 1:k_steps
% 系统状态空间方程，计算实际状态变量
    x = A * x + B * u + w(:,k);
% 计算实际测量值，添加了噪声（在实际应用中，这一项来自传感器测量）
    z = H_m * x + v(:,k);
% 使用卡尔曼滤波器
    [x_hat,x_hat_minus, P] = F8_LinearKalmanFilter(A,B,Q_c,R_c,H_m,z,x_hat,P,u);
% 保存系统状态到预先定义矩阵的相应位置
    x_history (:,k+1) =  x;
% 保存测量值到预先定义矩阵的相应位置
    z_history (:,k+1) =  z;   
% 保存先验估计到预先定义矩阵的相应位置
    x_hat_minus_history (:,k+1) = x_hat_minus;
% 保存后验估计到预先定义矩阵的相应位置
    x_hat_history (:,k+1) = x_hat;
end

%% 绘图
% figure(1)
% x1真实结果
plot (0:length(x_history)-1,x_history(1,:),'--','LineWidth',2);
hold on
% x1测量值
plot (0:length(z_history)-1,z_history(1,:),'*','MarkerSize',8)
hold on
% x1先验估计值
plot (0:length(x_hat_minus_history)-1,x_hat_minus_history(1,:),'o','MarkerSize',8);
hold on
% x1后验估计值
plot ( 0:length(x_hat_history)-1,x_hat_history(1,:),'LineWidth',2);
legend(' 真实值 ',' 测量值 ',' 先验估计值 ',' 后验估计值 ')
set(legend, 'Location', 'southeast','FontSize', 20);
ylim([-2 12]);
hold off;
grid on



