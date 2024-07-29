
function main
    % 定义系统参数
    m = 1;  % 摆杆质量
    M = 5;  % 小车质量
    L = 2;  % 摆杆长度
    g = -10;  % 重力加速度
    d = 1;  % 阻尼系数
    s = 1;  % 摆杆初始状态，1表示向上

    % 状态空间矩阵 A, B
    A = [0 1 0 0;
        0 -d/M -m*g/M 0;
        0 0 0 1;
        0 -s*d/(M*L) -s*(m+M)*g/(M*L) 0];
    B = [0; 1/M; 0; s*1/(M*L)];

    % 输出矩阵 C 和直接传递矩阵 D
    C = [0 0 1 0];  % 关心的输出是摆杆角度
    D = [0];

    % 导出传递函数
    [num,den] = ss2tf(A, B, C, D)
    sys_tf = tf(num, den)



    % % PID 控制器参数
    Kp = 100;
    Ki = 30;
    Kd = 25;

    % Kp = 1;
    % Ki = 0;
    % Kd = 0;

    % 时间跨度
    tspan = 0:.001:5;

    % 根据s值选择不同的初始条件
    if s == -1
        y0 = [0; 0; 0; 0];
    elseif s == 1
        y0 = [5; 0; pi + .1; 0];
    end

    % 使用 ODE 求解器进行模拟
    [t, y] = ode45(@(t, y) cartpend_pid(t, y, m, M, L, g, d, Kp, Ki, Kd), tspan, y0);

    % 可视化结果
    figure;
    for k = 1:100:length(t)
        drawcartpend(y(k,:), m, M, L);
        pause(0.01);  % 稍作暂停以便可视化动态
    end
end

function dy = cartpend_pid(t, y, m, M, L, g, d, Kp, Ki, Kd)
    persistent error_integral previous_error
    if isempty(error_integral)
        error_integral = 0;
        previous_error = 0;
    end

    desired_state = [1; 0; pi; 0];
    error = desired_state - y;
    error_integral = error_integral + error(3) * 0.001;  % 只积分角度误差
    derivative = (error(3) - previous_error) / 0.001;
    previous_error = error(3);

    % PID 控制器计算控制力
    u = Kp * error(3) + Ki * error_integral + Kd * derivative;

    % 系统动力学模型
    dy = cartpend(y, m, M, L, g, d, u);
end

function dy = cartpend(y, m, M, L, g, d, u)
    Sy = sin(y(3));
    Cy = cos(y(3));
    D = m*L*L*(M+m*(1-Cy^2));
    
    dy(1,1) = y(2);
    dy(2,1) = (1/D)*(-m^2*L^2*g*Cy*Sy + m*L^2*(m*L*y(4)^2*Sy - d*y(2))) + m*L*L*(1/D)*u;
    dy(3,1) = y(4);
    dy(4,1) = (1/D)*((m+M)*m*g*L*Sy - m*L*Cy*(m*L*y(4)^2*Sy - d*y(2))) - m*L*Cy*(1/D)*u;
end

function drawcartpend(y, m, M, L)
    x = y(1);
    th = y(3);

    W = 1*sqrt(M/5);  % 小车宽度
    H = 0.5*sqrt(M/5);  % 小车高度
    wr = 0.2;  % 轮子半径
    mr = 0.3*sqrt(m);  % 质量半径

    y_cart = wr/2 + H/2;  % 小车垂直位置
    w1x = x - 0.9*W/2;
    w2x = x + 0.9*W/2 - wr;
    w1y = 0;
    w2y = 0;

    px = x + L*sin(th);
    py = y_cart - L*cos(th);

    plot([-10 10], [0 0], 'k', 'LineWidth', 2);  % 绘制地面
    hold on;
    rectangle('Position', [x-W/2, y_cart-H/2, W, H], 'Curvature', 0.1, 'FaceColor', [1 0.1 0.1]);  % 小车
    rectangle('Position', [w1x, w1y, wr, wr], 'Curvature', 1, 'FaceColor', [0 0 0]);  % 轮子1
    rectangle('Position', [w2x, w2y, wr, wr], 'Curvature', 1, 'FaceColor', [0 0 0]);  % 轮子2
    plot([x px], [y_cart py], 'k', 'LineWidth', 2);  % 摆杆

    rectangle('Position', [px-mr/2, py-mr/2, mr, mr], 'Curvature', 1, 'FaceColor', [0.1 0.1 1]);  % 摆杆质量

    xlim([-6 6]);
    ylim([-2 3]);
    set(gcf, 'Position', [100 550 1000 400]);
    drawnow;
    hold off;
end
