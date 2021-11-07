%% Create map 
% start with blank map
dim = 150;
map = zeros(dim);

% the scaling of a pixel to actual length
scale = 10;

% let's start with a wall
map(1.8*scale : 2*scale,:) = 1;
heatmap(map,'GridVisible', 'off', 'XDisplayLabels',nan(dim,1),'YDisplayLabels',nan(dim,1))
%% inference map

% create map (log likelihood) and inference parameter
m = zeros(dim);
param = [0.02, 0.2, 5, 0.1, 0.05];

% start with this position
x = [2.5, 7.5, pi/2];

% start simulation
for u=0:11
%     xi = rand * 2*pi;
%     if x(1)+cos(x(3)+xi) < 42
%         xi = xi + pi;
%     end
%     next = [x(1)+cos(x(3)+xi), x(2)+sin(x(3)+xi), x(3)+xi]; next = x;

    % move horizontally 
    next = x + [0, 1, 0];
    
    % at every step, send out beams in every direction
    for angle = 0:0.05:2*pi
        % tune angle to be in (0,2pi)
        theta = rem(angle + x(3), 2*pi);
        
        % the shortest distacne from position to wall 
        shortest_distance = x(1) - 2;

        % find distance to wall in the bearing angle
        if 0 < theta && theta < pi/2 || 3*pi/2 < theta && theta < 2*pi
            true_distance = param(3)*2;
        else
            true_distance = min(abs(shortest_distance/cos(theta)),param(3));
        end

        % scan! 
        z = gen_sensor_measurement(true_distance, param);
        
        % now we can get our belief from the path the beam went through
        t = 0:0.005:z+0.05;
        y = [x(1) + t*cos(theta); x(2) + t*sin(theta)];
        % for each length in the path, update belief in the column
        for i=1:length(y(1,:))
            r = sqrt((x(1)-y(1,i))^2 + (x(2)-y(2,i))^2); % compute distance
            p = belief_assignment(r,z,[1,0.3],param); % calculate belief
            logodds = log(p/(1-p)); % calculate log odds
            % make sure no errors from reading errors and update belief
            if ceil(y(1,i)*scale) < dim && ceil(y(2,i)*scale) < dim && ceil(y(1,i)*scale) > 0 && ceil(y(2,i)*scale) > 0
                m(ceil(y(1,i)*scale),ceil(y(2,i)*scale)) = m(ceil(y(1,i)*scale),ceil(y(2,i)*scale)) + logodds;
            end
        end
    end
    x = next;
end

% get map into probabilty
m = 1-1./(1+exp(m));
%% show final map
heatmap(m,'GridVisible', 'off', 'XDisplayLabels',nan(dim,1),'YDisplayLabels',nan(dim,1))