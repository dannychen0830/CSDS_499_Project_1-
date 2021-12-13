function [J,U] = value_iteration(M,P,eps,gamma)

M_ = find(M~=0);

J = zeros(1,length(M));
U = zeros(1,length(M));
J(M==1) = 1;

delta = 100;
J_ = zeros(1,length(J));
count = 0;
while delta > eps
    
    for k=1:length(M_)
        j = M_(k);
        ov = J(j);
        nv = ov;
        for u=1:size(P,1)
            T = reshape(P(u,j,:),1,length(J));
            v = sum(T.*(-gamma + J));
%                 nv = max([v, nv]);
            if v > nv
                nv = v;
                U(j) = u;
            end
        end
        J(j) = nv;
    end
    delta = norm(J-J_);
    J_ = J;
%     disp(delta)
    if rem(count,100) == 0
        disp(delta)
    end
    count = count + 1;
    
end