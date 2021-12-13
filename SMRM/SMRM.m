%% Sample configurations

n = 2000;
V = zeros(3,n+1); V(:,1) = [5,2,pi/2]; V(:,2) = [6.5,7.5,pi/4]; V(:,n+1) = [-1;-1;0];

nn = n;
while nn > 2
    q = [rand*9, rand*9, rand*2*pi];
    if isCollisionFree(q)
        V(:,nn) = q;
        nn = nn-1;
    end
end

scatter(V(1,:),V(2,:))
hold on
scatter(V(1,2),V(2,2),'r')
hold on
scatter(V(1,1),V(2,1),'g')

%% build transition
P = zeros(4,n+1,n+1);
m = 20;
alpha = 2;

for s=1:n
    for u=1:2
%         dest = zeros(1,m);
        for j=1:m
            [q,path] = generateSampleTransition(V(:,s),u);
            if isCollisionFreePath(path)
                V_ = V(:,1:n); V_(3,:) = alpha*rem(V_(3,:) + 2*pi, 2*pi);
                q_ = q; q_(3) = alpha*q_(3);
                dist = zeros(1,n);
                for i=1:n
                    dist(i) = norm(V_(:,i)-q_');
                end
                [z,t] = min(dist);
            else
                t = n+1;
            end
%             dest(j) = t;
            P(u,s,t) = P(u,s,t) + 1/m;
        end
%         for t=dest
%             P(u,s,dest) = P(u,s,dest) + 1/m;
%         end
    end
end

%%
G = digraph(reshape(P(1,:,:),n+1,n+1));
plot(G,'NodeLabel','','XData',V(1,:),'YData',V(2,:))

%%
% x = [1,1]; 
x = [1,1,pi/2];
[q,p] = generateSampleTransition(x,1);
disp(q)
plot(p(1,:),p(2,:))

%%
disp(isCollisionFree([1,8,pi/2],bw))

%% MDP 
gamma = 1e-6; eps=1e-6;
M = ones(1,n+1)*0.5; M(2) = 1; M(n+1) = 0;
for i=1:n
    if norm(V(1:2,i) - V(1:2,2)) < r
    	M(i) = 1;
    end
end
[J,U] = value_iteration(M,P,eps,gamma);

%% Try the optimal path 
n = 2000;
bw = 0.17;
m = 20;
alpha=2;
r = .5;
gamma = 1e-8;
eps = 1e-10;

[V,P,U] = runSMRM(n,bw,m,alpha,r,gamma,eps);

% Visualize Path

plzstop = 0;

t = 0:0.1:1;
plot(2+(4.5-bw)*t,4*ones(1,11),'k','linewidth',1.5) 
hold on
plot(2+(4.5-bw)*t,6*ones(1,11),'k','linewidth',1.5) 
hold on
plot((6.5-bw)*ones(1,11),4+2*t,'k','linewidth',1.5)
hold on
plot(2*ones(1,11),4+2*t,'k','linewidth',1.5)
hold on
plot((6.5+bw)*ones(1,11),4+2*t,'k','linewidth',1.5)
hold on
plot((6.5+bw)+2*t,4*ones(1,11),'k','linewidth',1.5)
hold on
plot((6.5+bw)+2*t,6*ones(1,11),'k','linewidth',1.5)
hold on
t = 0:0.1:2*pi;

plot(6.5+r*cos(t), 7.5+r*sin(t),'g','linewidth',1.5)
hold on
xlim([0 9])
ylim([0 9])

for run=1:10
    x = V(:,1); ind = 1;
    while ~plzstop
        [p, path] = generateSampleTransition(x, U(ind));
        x = p;
        plot(path(1,:),path(2,:),'color',[0, 0.4470, 0.7410],'linewidth',1.5)
        hold on
        pause(1)

        if isCollisionFreePath(path,bw)
            V_ = V(:,1:n); %V_(3,:) = alpha*rem(V_(3,:) + 2*pi, 2*pi);
            p_ = p; %p_(3) = alpha*p_(3);
            dist = zeros(1,n);
            for i=1:n
                dist(i) = norm(V_(:,i)-p_');
            end
            [z,ind] = min(dist);
        else
            ind = n+1;
        end

        if ind == n+1
            plzstop = 1;
            plot(path(1,:),path(2,:),'r','linewidth',1.5)
            hold on
            disp('ughhhhh')
        end

        if norm(p(1:2)' - V(1:2,2)) < r
            plzstop = 1;
            disp('yayyy')
        end
    end
    plzstop = 0;
end
