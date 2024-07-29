clear all, close all, clc

m = 1;
M = 5;
L = 2;
g = -10;
d = 1;

s = 1; % pendulum up (s=1)

A = [0 1 0 0;
    0 -d/M -m*g/M 0;
    0 0 0 1;
    0 -s*d/(M*L) -s*(m+M)*g/(M*L) 0];

B = [0; 1/M; 0; s*1/(M*L)];

% 选择摆角为输出
C = [0 0 1 0];

% 直接传递矩阵D通常为0
D = [0];

[num, den] = ss2tf(A, B, C, D);
sys_tf = tf(num, den);

% 打印传递函数
disp(sys_tf)

% 绘制系统的阶跃响应
step(sys_tf)

% 绘制系统的频率响应
% bode(sys_tf)

% 系统稳定性分析
% pole(sys_tf)

% 使用 pidtune 自动调整 PID 参数
[controller, info] = pidtune(sys_tf, 'PID');

closed_loop_sys = feedback(controller * sys_tf, 1);

% 阶跃响应
t = 0:0.01:10;  % 时间向量
[y, t] = step(closed_loop_sys, t);

% 绘制响应
figure;
for k = 1:length(t)
    drawcartpend([y(k); 0; pi; 0], 1, 5, 2);  % 假定 y(k) 是摆角的偏移量
    pause(0.05);  % 暂停以可视化动画
end
