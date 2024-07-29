clear

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
C = [1 0 0 0];

% 直接传递矩阵D通常为0
D = [0];
[num, den] = ss2tf(A, B, C, D);
sys_tf = tf(num, den);

% 定义 PID 参数
Kp = 100;
Ki = 1;
Kd = 20;

% 创建 PID 控制器
pid_controller = pid(Kp, Ki, Kd);

% 构建闭环系统
closed_loop_sys = feedback(pid_controller * sys_tf, 1);

% 模拟阶跃响应并绘图
figure;
step(closed_loop_sys);
title(sprintf('Step Response with Kp = %g, Ki = %g, Kd = %g', Kp, Ki, Kd));
grid on;