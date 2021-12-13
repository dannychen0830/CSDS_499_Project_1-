function [V,P,U] = runSMRM(n,bw,m,alpha,r,gamma,eps)

% V = zeros(3,n+1); V(:,1) = [5,2,pi/2]; V(:,2) = [6.5,7.5,pi/4]; V(:,n+1) = [-1;-1;0];
V = zeros(2,n+1); V(:,1) = [5,2]; V(:,2) = [6.5,7.5]; V(:,n+1) = [-1;-1];
nn = n;
while nn > 2
    q = [rand*9, rand*9];
    if isCollisionFree(q,bw)
        V(:,nn) = q;
        nn = nn-1;
    end
end


disp('Building Transition')
P = zeros(4,n+1,n+1);
for s=1:n
    for u=1:4
%         dest = zeros(1,m);
        for j=1:m
            [q,path] = generateSampleTransition(V(:,s),u);
            if isCollisionFreePath(path,bw)
                V_ = V(:,1:n); % V_(3,:) = alpha*rem(V_(3,:) + 2*pi, 2*pi);
                q_ = q; % q_(3) = alpha*q_(3);
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

disp('Running Value Iteration')
M = ones(1,n+1)*0.5; M(2) = 1; M(n+1) = 0;
% for i=1:n
%     if norm(V(1:2,i) - V(1:2,2)) < r
%     	M(i) = 1;
%     end
% end
[J,U] = value_iteration(M,P,eps,gamma);

end

