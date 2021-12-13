%% Set up: Lattice, Transition-matrix, Reward Function

n = 100; N = n*n; % size of lattice

p = 0.7; % probability of moving the right way 
q = (1-p)/3;

gamma = 0.0001; % punishment

A = zeros(N); % adjacency matrix
P = zeros(4,N,N); % transition probabilities
J = ones(1,N)*0.5; J(N) = 1; % probability of success


for k=1:N
    i = floor((k-1)/n)+1;
    j = rem(k,n);
    if j == 0
        j = n;
    end
    
    for u=1:4
        if j < n
            A(k,k+1) = 1;
            P(u,k,k+1) = (u==1)*p + (u~=1)*q;
        end
        if j > 1
            A(k,k-1) = 1;
            P(u,k,k-1) = (u==2)*p + (u~=2)*q;
        end
        if i < n
            A(k,i*n+j) = 1;
            P(u,k,i*n+j) = (u==3)*p + (u~=3)*q;
        end
        if i > 1
            A(k,(i-2)*n+j) = 1;
            P(u,k,(i-2)*n+j) = (u==4)*p + (u~=4)*q;
        end
    end
    
    if j+2 < i
        J(k) = 0;
    end
end

G = graph(A);
% plot(G,'NodeLabel',J)

%% optimize

[J_,U] = value_iteration(J,P,0.0001,gamma);
% plot(G,'NodeLabel',J_)


