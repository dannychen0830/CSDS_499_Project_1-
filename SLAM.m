%% U-shaped Map as always

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

%% SLAM! Initialization

np = 500; % number of particles 
param = [0.02, 0.02, 5, 0.1, 0.1]; % parameters for sensor model
alpha = [0.01, 0.002, 0.0001, 0.0001]; % motion noise parameter

% initialize robot
x = zeros(length(motion), np, 3); % position after localization
y = zeros(length(motion), np, 3); % position before localization
m = zeros(length(motion), np, dim, dim); % maps
for i=1:np, x(1,i,:) =  [dim/scale*rand, dim/scale*rand, pi/2+normrnd(0,1)];y(1,i,:) = x(1,i,:); end
%  x(1,i,:) = motion(1,:) + normrnd(0,1,1,3);

t = 1; % first time step
num_sensor = 200; % number of beams 
angle = 0:2*pi/num_sensor:2*pi; % num_sensor equally spaced w.r.t angles
z = zeros(1,length(angle)); 

% get true measurement 
% start scanning
for i=1:length(angle)
    % tune angle to be in (0,2pi)
    theta = rem(angle(i) + motion(t+1,3), 2*pi);

    % we know the map, so compute the true distance
    if 0 <= theta && theta < pi/2 
        true_distance = min( (12 - motion(t+1,1))/cos(theta), (12-motion(t+1,2))/sin(theta) ); 
    elseif pi/2 <= theta && theta < pi
        true_distance = min( (motion(t+1,1)-2)/cos(pi-theta), (12-motion(t+1,2))/sin(pi-theta) ); 
    elseif pi <= theta && theta < 3*pi/2
        true_distance = (motion(t+1,1) - 2)/cos(theta);
    else
        true_distance = (12 - motion(t+1,1))/cos(2*pi - theta);
    end
    true_distance = abs(true_distance);

    % scan! 
    z(i) = gen_sensor_measurement(true_distance, param);
end

% now we can get our belief from the path the beam went through
for n=1:np
    pos = x(t,n,:);
    r = 0:0.005:z+0.05;
    for j=1:length(angle)
        ray = [pos(1) + r*cos(pos(3)+angle(j)); pos(2) + r*sin(pos(3)+angle(j))];
        % for each length in the path, update belief in the column
        for i=1:length(ray(1,:))
            rr = sqrt((pos(1)-ray(1,i))^2 + (pos(2)-ray(2,i))^2); % compute distance
            p = belief_assignment(rr,z(j),[1,0.3],param); % calculate belief
            logodds = log(p/(1-p)); % calculate log odds
            % make sure no errors from reading errors and update belief
            if ceil(ray(1,i)*scale) < dim && ceil(ray(2,i)*scale) < dim && ceil(ray(1,i)*scale) > 0 && ceil(ray(2,i)*scale) > 0
                m(t,n,ceil(ray(1,i)*scale),ceil(ray(2,i)*scale)) = m(t,n,ceil(ray(1,i)*scale),ceil(ray(2,i)*scale)) + logodds;
            end
        end
    end
end

% SLAM! Propogate

for t=1:length(motion)-1
    
    w = ones(1,np); % store wegiths
    next = zeros(np, 3); % store particle of next step
    
    % get measurement (beam with noiseless rotation)
    % start scanning
    for i=1:length(angle)
        % tune angle to be in (0,2pi)
        theta = rem(angle(i) + motion(t+1,3), 2*pi);
        
        % we know the map, so compute the true distance
        if 0 <= theta && theta < pi/2 
            true_distance = min( (12 - motion(t+1,1))/cos(theta), (12-motion(t+1,2))/sin(theta) ); 
        elseif pi/2 <= theta && theta < pi
            true_distance = min( (motion(t+1,1)-2)/cos(pi-theta), (12-motion(t+1,2))/sin(pi-theta) ); 
        elseif pi <= theta && theta < 3*pi/2
            true_distance = (motion(t+1,1) - 2)/cos(theta);
        else
            true_distance = (12 - motion(t+1,1))/cos(2*pi - theta);
        end
        true_distance = abs(true_distance);

        % scan! 
        z(i) = gen_sensor_measurement(true_distance, param);
    end

    % sample from motion model and compute weight for each particle
    for j=1:np
        next(j,:) = sample_motion_model_odometry([motion(t,:),motion(t+1,:)],x(t,j,:),alpha); 
        
        
        
        mm = reshape(m(t,j,:,:),dim,dim);
        mm = 1-1./(1+exp(mm));
        mm = mm > 1/2; %mm = mm < 1/2; rand(dim,dim);
        mm(:,1:2) = 1; mm(:,dim-2:dim) = 1; mm(1:2,:) = 1; mm(dim-2:dim,:) = 1;
        
        if j == 1
            figure(1)
            imagesc(mm)
            pause(1)
        end
        
        % given the bearing angle and map, find the probability of measurement
        for a=1:length(angle)
            % extend r until hit an obstacle
            for r=0:0.01:param(3)+1/scale
                
                % convert to map index
                new_x = next(j,1) + r*cos(next(j,3)+angle(a)); new_y = next(j,2) + r*sin(next(j,3)+angle(a));
                new_x = floor(new_x*scale); new_y = floor(new_y*scale);
                
                % this is avoiding boundary conditions
                if next(j,1) < 0 || next(j,1) > dim/scale || next(j,2) > dim/scale || next(j,2) < 0
                    w(j) = w(j)*0.0001;
                    break 
                end            
                if new_x <= 0 || new_y <= 0 || new_x > dim || new_y > dim
                    w(j) = w(j)*0.0001;
                    break
                end
                
                % if I hit an obstacle, I calculate the probability (*10 for numerical stability)
                if mm(new_x, new_y) == 1
                    w(j) = w(j)*5*prob_sensor_measurement(z(a),r,param);
                    break
                end
            end
            
            % if I didn't hit an obstacle, max reading is reached
            if r > param(3)
                w(j) = w(j)*10*prob_sensor_measurement(z(a),r,param);
            end 
        end
    end
    
    % store result
    y(t+1,:,:) = next;
    
    % resample states according to weight
    figure(2)
    w = w/sum(w);
    for j=1:np
        i = sample_state(w);
        x(t+1,j,:) = next(i,:);
        m(t,j,:,:) = m(t,i,:,:);
    end
    
    % plot result
    scatter(y(t,:,1),y(t,:,2),'b')
    hold on
    scatter(x(t,:,1),x(t,:,2),'r')
    hold off
    xlim([1 dim/scale])
    ylim([1 dim/scale])
    pause(2)
    
    for n=1:np
        m(t+1,n,:,:) = m(t,n,:,:);
        pos = x(t+1,n,:);
        r = 0:0.005:z+0.05;
        for j=1:length(angle)
            ray = [pos(1) + r*cos(pos(3)+angle(j)); pos(2) + r*sin(pos(3)+angle(j))];
            % for each length in the path, update belief in the column
            for i=1:length(ray(1,:))
                rr = sqrt((pos(1)-ray(1,i))^2 + (pos(2)-ray(2,i))^2); % compute distance
                p = belief_assignment(rr,z(j),[1,0.3],param); % calculate belief
                logodds = log(p/(1-p)); % calculate log odds
                % make sure no errors from reading errors and update belief
                if ceil(ray(1,i)*scale) < dim && ceil(ray(2,i)*scale) < dim && ceil(ray(1,i)*scale) > 0 && ceil(ray(2,i)*scale) > 0
                    m(t+1,n,ceil(ray(1,i)*scale),ceil(ray(2,i)*scale)) = m(t+1,n,ceil(ray(1,i)*scale),ceil(ray(2,i)*scale)) + logodds;
                end
            end
        end
    end
end
    

