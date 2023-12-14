% 《控制之美-卷二》 P95-96
% 求解令 J = 0.5*(U1.^2 + U2.^2)+U1+U2 最小时的u1、u2
clear;
close all;
clc;

%% 转化为标准二次规划问题
% J = (1/2)(u^T)Hu + (uT)f
% 定义二次规划问题的H和f
H = [1 0; 0 1];
f = [1; 1];

% 求解二次规划问题
% 可以轻易求出无约束情况下的解析解：
u = -inv(H)*f;



%% 绘图
% 绘制二次规划问题的可行域和最优解点（3D图）
[U1,U2] = meshgrid(-2:0.1:0);
J = 0.5*(U1.^2 + U2.^2)+U1+U2;
subplot(1,2,1);
surf(U1,U2,J,'FaceAlpha', 0.1);
hold on;
plot3(u(1), u(2), 0.5*(u(1)^2 + u(2)^2)+u(1)+u(2), 'r^', 'MarkerSize', 20,'MarkerFaceColor', 'red');
xlabel('u1');
ylabel('u2');
zlabel('J(u1,u2)');
xlim([-2 0]);
ylim([-2 0]);
zlim([-1.05 0]);
set(gca,'FontSize',20);
% 绘制等高线图
subplot(1,2,2);
contour(U1,U2,J,30);
hold on;
plot(u(1), u(2), 'r*', 'MarkerSize', 10);
xlabel('u1');
ylabel('u2');
set(gca,'FontSize',20);
sgtitle('二次规划问题');
