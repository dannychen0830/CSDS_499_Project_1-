%% Create map 
% start with blank map
map = zeros(100);

% let's start with a wall
map(38:42,:) = 1;

%% inference map
m = zeros(100);
param = [0.02, 0.2, 5, 0.1, 0.1];

x = [45, 50, pi/2];

d = 3;

for j = 1:10
    for i=1:3
        z = gen_sensor_measurement(d, param);
        p = belief_assignment(z,i,param);

        logodds = log(p/(1-p));
        m(45-i,50) = m(45-i,50) + logodds;
    end
end

%%
heatmap(m)