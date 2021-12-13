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
param = [0.02, 0.02, 5, 0.1, 0.05];

% start with this position
x = [6, 7.5, pi/2];

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
    for angle = 0:0.01:2*pi
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

%% A U-shaped map 
dim = 150;
map = zeros(dim);

% the scaling of a pixel to actual length
scale = 10;

% create the U-shape wall
map(1.8*scale : 2*scale,:) = 1;
map(:, 12*scale : 12.2*scale) = 1;
map(12*scale:12.2*scale, :) = 1;
map(:,1:2) = 1; map(:,dim-2:dim) = 1; map(1:2,:) = 1; map(dim-2:dim,:) = 1; % set walls on boundary

% create the movements 
motion = [6, 7, pi/2; 6, 8, pi/2; 6, 9, pi/2; 6, 10, 0; 7, 10, 0; 8, 10, 0;  9, 10, -pi/2; 9, 9, -pi/2; 9, 8, -pi/2; 9, 7, -pi/2];

for i=1:length(motion), map(scale*motion(i,1),scale*motion(i,2)) = 2; end
imagesc(map)%,'GridVisible', 'off', 'XDisplayLabels',nan(dim,1),'YDisplayLabels',nan(dim,1))
axis off
% scatter(motion(:,1),motion(:,2),'filled')
%%  inference

% create map (log likelihood) and inference parameter
m = zeros(dim);
param = [0.05, 0.02, 5, 0.1, 0.1];

% start simulation
for u=1:length(motion)
    x = motion(u,:);
    disp(x)
    
    % at every step, send out beams in every direction
    for angle = 0:0.01:2*pi
        % tune angle to be in (0,2pi)
        theta = rem(angle + x(3), 2*pi);
        
        % find the closest distance by searching through the beam
        
        true_distance = 1000;
        if 0 <= theta && theta < pi/2 
            true_distance = min( (12 - x(1))/cos(theta), (12-x(2))/sin(theta) ); 
        elseif pi/2 <= theta && theta < pi
            true_distance = min( (x(1)-2)/cos(pi-theta), (12-x(2))/sin(pi-theta) ); 
        elseif pi <= theta && theta < 3*pi/2
            true_distance = (x(1) - 2)/cos(theta);
        else
            true_distance = (12 - x(1))/cos(2*pi - theta);
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
end

% get map into probabilty
m = 1-1./(1+exp(m));

%% show final map
for i=1:length(motion), m(scale*motion(i,1),scale*motion(i,2)) = 2; end
imagesc(m)%,'GridVisible', 'off', 'XDisplayLabels',nan(dim,1),'YDisplayLabels',nan(dim,1))
axis off
