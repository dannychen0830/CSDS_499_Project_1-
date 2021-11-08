% start with this position
T = 10; %测量时间(总步数)
R = 10; %测量噪声
par_num = 1000;
X = zeros(3, T); %存储系统状态(每列存储二维位置坐标(x,y,0),共T个位置)
Z = zeros(2, T); %存储系统的观测状态(每列存储二维位置坐标(x,y),共T次测量)
P = zeros(3,par_num);
w = zeros(par_num,1);

X(:,1) = [6,7.5,pi/2];
WorldSize = 10; %世界大小
u = [6, 7, pi/2; 6, 8, pi/2; 6, 9, pi/2; 6, 10, 0; 7, 10, 0; 8, 10, 0; 9, 10, 3*pi/2; 9, 9, 3*pi/2; 9, 8, 3*pi/2; 9, 7, 3*pi/2];

% 从第二步开始
for k = 2:T
    % sample预期的状态
    u_t = u(k-1,:);
    alpha = [0.0001,0.0001,0.0001,0.0001,0.0001,0.0001];
    X(:, k) = sample_motion_model_odemetry(u,X(:, k-1),alpha); %状态方程
    
    % param = [0.0200, 0.2000, 5.0000, 0.1000, 0.1000];
    % 得到_sensor _measurement
    Z(:, k) = X(1:2, k) + wgn(2, 1, 10*log10(R)); %观测方程(状态上叠加测量的高斯噪声) 
                             %y = wgn(m,n,p) 产生一个m行n列的高斯白噪声的矩阵，p以dBW为单位指定输出噪声的强度
    
    % 每一个particle计算预期状态，测量与Z的距离，权重大的留下小的舍去，进行重采样
    for i = 1:par_num
        P(:, i) = [WorldSize*rand; WorldSize*rand;pi*rand];%随机产生第i个粒子的坐标(rand为产生[0,1]之间均匀分布)s
        P(:, i) = sample_motion_model_odemetry(u,P(:, i),alpha);%粒子群带入状态方程
        dist = norm(P(1:2, i)-Z(1:2, k)); %粒子群中各粒子 与 测量位置 的距离
        w(i) = (1 / sqrt(R) / sqrt(2 * pi)) * exp(-(dist)^2 / 2 / R); %求权重(距离近权重大)
    end
    %归一化权重
    wsum = sum(w);
    for i = 1 : par_num
        w(i) = w(i) / wsum;
    end
    %重采样（更新）
    for i = 1 : par_num
        wmax = 2 * max(w) * rand; %另一种重采样规则
        index = randi(par_num, 1);%生成一个在[1(默认值),N]之间均匀分布的伪随机整数
        while(wmax > w(index))
            wmax = wmax - w(index);
            index = index + 1;
            if index > par_num
                index = 1;
            end 
        end
        Pnext(1:2, i) = P(1:2, index); %得到新粒子放入临时集Pnext
    end
    P(1:2)=Pnext(1:2);%用临时集Pnext更新粒子集P
    
    figure(2);
    set(0,'defaultfigurecolor','w')
    clf;%清空figure(2)中的图像 以便循环重新画
    plot(X(1, 1), X(2, 1), 'r.', 'markersize',30) %真实的初始状态位置(红点表示)
    clf;
    hold on
    plot(X(1, k), X(2, k), 'g.', 'markersize',30); %系统状态位置
    plot(P(1, :), P(2, :), 'k.', 'markersize',5); %各个粒子位置
    axis([0 10 0 10]);
    title('运动过程');
    legend('真实状态', '粒子群');
    hold off
    pause(0.5);%停0.1s开始下次迭代
end
%% 绘制轨迹
figure(3);
set(0,'defaultfigurecolor','w')
plot(X(1,:), X(2,:), 'r.-', Z(1,:), Z(2,:), 'g.-');
axis([0 10 0 10]);
set(gca,'XTick',0:10:100) %改变x轴坐标间隔显示 这里间隔为10
set(gca,'YTick',0:10:100) %改变y轴坐标间隔显示 这里间隔为10
legend('真实轨迹', '测量轨迹');
xlabel('横坐标 x'); ylabel('纵坐标 y');

    
    
