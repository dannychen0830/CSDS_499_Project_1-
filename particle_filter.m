% start with this position
T = 10; %����ʱ��(�ܲ���)
R = 10; %��������
par_num = 1000;
X = zeros(3, T); %�洢ϵͳ״̬(ÿ�д洢��άλ������(x,y,0),��T��λ��)
Z = zeros(2, T); %�洢ϵͳ�Ĺ۲�״̬(ÿ�д洢��άλ������(x,y),��T�β���)
P = zeros(3,par_num);
w = zeros(par_num,1);

X(:,1) = [6,7.5,pi/2];
WorldSize = 10; %�����С
u = [6, 7, pi/2; 6, 8, pi/2; 6, 9, pi/2; 6, 10, 0; 7, 10, 0; 8, 10, 0; 9, 10, 3*pi/2; 9, 9, 3*pi/2; 9, 8, 3*pi/2; 9, 7, 3*pi/2];

% �ӵڶ�����ʼ
for k = 2:T
    % sampleԤ�ڵ�״̬
    u_t = u(k-1,:);
    alpha = [0.0001,0.0001,0.0001,0.0001,0.0001,0.0001];
    X(:, k) = sample_motion_model_odemetry(u,X(:, k-1),alpha); %״̬����
    
    % param = [0.0200, 0.2000, 5.0000, 0.1000, 0.1000];
    % �õ�_sensor _measurement
    Z(:, k) = X(1:2, k) + wgn(2, 1, 10*log10(R)); %�۲ⷽ��(״̬�ϵ��Ӳ����ĸ�˹����) 
                             %y = wgn(m,n,p) ����һ��m��n�еĸ�˹�������ľ���p��dBWΪ��λָ�����������ǿ��
    
    % ÿһ��particle����Ԥ��״̬��������Z�ľ��룬Ȩ�ش������С����ȥ�������ز���
    for i = 1:par_num
        P(:, i) = [WorldSize*rand; WorldSize*rand;pi*rand];%���������i�����ӵ�����(randΪ����[0,1]֮����ȷֲ�)s
        P(:, i) = sample_motion_model_odemetry(u,P(:, i),alpha);%����Ⱥ����״̬����
        dist = norm(P(1:2, i)-Z(1:2, k)); %����Ⱥ�и����� �� ����λ�� �ľ���
        w(i) = (1 / sqrt(R) / sqrt(2 * pi)) * exp(-(dist)^2 / 2 / R); %��Ȩ��(�����Ȩ�ش�)
    end
    %��һ��Ȩ��
    wsum = sum(w);
    for i = 1 : par_num
        w(i) = w(i) / wsum;
    end
    %�ز��������£�
    for i = 1 : par_num
        wmax = 2 * max(w) * rand; %��һ���ز�������
        index = randi(par_num, 1);%����һ����[1(Ĭ��ֵ),N]֮����ȷֲ���α�������
        while(wmax > w(index))
            wmax = wmax - w(index);
            index = index + 1;
            if index > par_num
                index = 1;
            end 
        end
        Pnext(1:2, i) = P(1:2, index); %�õ������ӷ�����ʱ��Pnext
    end
    P(1:2)=Pnext(1:2);%����ʱ��Pnext�������Ӽ�P
    
    figure(2);
    set(0,'defaultfigurecolor','w')
    clf;%���figure(2)�е�ͼ�� �Ա�ѭ�����»�
    plot(X(1, 1), X(2, 1), 'r.', 'markersize',30) %��ʵ�ĳ�ʼ״̬λ��(����ʾ)
    clf;
    hold on
    plot(X(1, k), X(2, k), 'g.', 'markersize',30); %ϵͳ״̬λ��
    plot(P(1, :), P(2, :), 'k.', 'markersize',5); %��������λ��
    axis([0 10 0 10]);
    title('�˶�����');
    legend('��ʵ״̬', '����Ⱥ');
    hold off
    pause(0.5);%ͣ0.1s��ʼ�´ε���
end
%% ���ƹ켣
figure(3);
set(0,'defaultfigurecolor','w')
plot(X(1,:), X(2,:), 'r.-', Z(1,:), Z(2,:), 'g.-');
axis([0 10 0 10]);
set(gca,'XTick',0:10:100) %�ı�x����������ʾ ������Ϊ10
set(gca,'YTick',0:10:100) %�ı�y����������ʾ ������Ϊ10
legend('��ʵ�켣', '�����켣');
xlabel('������ x'); ylabel('������ y');

    
    
