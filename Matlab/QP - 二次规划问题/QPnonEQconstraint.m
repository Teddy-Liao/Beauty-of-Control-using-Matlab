% 《控制之美-卷二》 P97-98
% 求解令 J = 0.5*(U1.^2 + U2.^2)+U1+U2 最小时的u1、u2
% 同时满足不等式约束
% u1  + u2  <= 2
% -u1 + u2  <= 1
% 0 =< u1 <= 1
% 0 =< u2 <= 2

clear all;
close all;
clc;

% 定义二次规划问题的H和f
H = [1 0; 0 1];
f = [1; 1];

% 定义不等式约束的A和b
A = [-1 1; 1 1];
b = [1; 2];
% 定义变量的边界条件
lb = [0; 0];
ub = [1; 2];

% 使用 quadprog求解器求解含不等式约束的二次规划问题
% 使用格式 [x,fval] = quadprog(H,f,A,b,Aeq,beq,lb,ub,x0,options)
% fval 是标准的代价函数
% fval = 0.5*x'*H*x + f'*x
[u, J] = quadprog(H, f, A, b, [], [], lb, ub, []);

%% 绘图
% 绘制等高线图
subplot(1,1,1);
[U1, U2] = meshgrid(-1:0.1:2);
J = 0.5*(U1.^2 + U2.^2) + U1 + U2;
contour(U1, U2, J, 60);
hold on;
% 绘制可行域
plot([-0.5, 1], [0.5, 2], 'k', 'LineWidth', 1.5);
hold on;
plot([0, 1.5], [2, 0.5], 'k', 'LineWidth', 1.5);
plot([0, 0], [-1, 2], 'k', 'LineWidth', 1.5);
plot([-0.5, 1.5], [0, 0], 'k', 'LineWidth', 1.5);
plot([1, 1], [-1, 2], 'k', 'LineWidth', 1.5);
fill([0, 0.5, 1], [1, 1.5, 1], 'green', 'FaceAlpha', 0.2, 'EdgeColor', 'none');
fill([0, 0, 1, 1], [0, 1, 1, 0], 'green', 'FaceAlpha', 0.2, 'EdgeColor', 'none');
% 绘制最优解点
plot(u(1), u(2), 'r^', 'MarkerSize', 20,'MarkerFaceColor', 'red');

% 添加坐标轴标签和图标题
xlabel('u1');
ylabel('u2');
xlim([-0.5 1.5]);
ylim([-0.5 2]);
set(gca, 'FontSize', 20);