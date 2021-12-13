%% create map 

dim = 150;
map = zeros(dim);

% the scaling of a pixel to actual length
scale = 10;

% create the U-shape wall
map(1.8*scale : 2*scale,:) = 1;
map(:, 12*scale : 12.2*scale) = 1;
map(12*scale:12.2*scale, :) = 1;
map(:,1:2) = 1; map(:,dim-2:dim) = 1; map(1:2,:) = 1; map(dim-2:dim,:) = 1; % set walls on boundary
imagesc(map)%,'GridVisible', 'off', 'XDisplayLabels',nan(dim,1),'YDisplayLabels',nan(dim,1))

% create the movements (use this as the actual motion, too)
motion = [6, 7, pi/2; 6, 8, pi/2; 6, 9, pi/2; 6, 10, 0; 7, 10, 0; 8, 10, 0;  9, 10, -pi/2; 9, 9, -pi/2; 9, 8, -pi/2; 9, 7, -pi/2];

%% Particle filter

np = 1000; % number of particles 
param = [0.05, 0.02, 5, 0.1, 0.1]; % parameters for sensor model
alpha = [0.01, 0.002, 0.0001, 0.0001]; % motion noise parameter

% initialize robot
x = zeros(length(motion), np, 3);
y = zeros(length(motion), np, 3);
for i=1:np, x(1,i,:) =  [dim/scale*rand, dim/scale*rand, pi/2]; y(1,i,:) = x(1,i,:); end

% start motion
for t=1:length(motion)-1
    
    w = ones(1,np); % store wegiths
    next = zeros(np, 3); % store particle of next step
    
    % get measurement (beam with noiseless rotation)
    num_sensor = 10; % number of beams 
    angle = 0:2*pi/num_sensor:2*pi; % num_sensor equally spaced w.r.t angles
    z = zeros(1,length(angle)); 
    
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

        % given the bearing angle and map, find the probability of measurement
        for a=1:length(angle)
            % extend r until hit an obstacle
            for r=0:0.01:param(3)+1/scale
                
                % convert to map index
                new_x = next(j,1) + r*cos(next(j,3)+angle(a)); new_y = next(j,2) + r*sin(next(j,3)+angle(a));
                new_x = floor(new_x*scale); new_y = floor(new_y*scale);
                
                % this is avoiding boundary conditions
                if next(j,1) < 0 || next(j,1) > dim/scale || next(j,2) > dim/scale || next(j,2) < 0
                    w(j) = 0;
                    break 
                end            
                if new_x == 0 || new_y == 0
                    w(j) = 0;
                    break
                end
                
                % if I hit an obstacle, I calculate the probability (*10 for numerical stability)
                if map(new_x, new_y) == 1
                    w(j) = w(j)*10*prob_sensor_measurement(z(a),r,param);
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
    w = w/sum(w);
    for j=1:np
        i = sample_state(w);
        x(t+1,j,:) = next(i,:);
    end
    
    % plot result
    scatter(y(t,:,1),y(t,:,2),'b')
    hold on
    scatter(x(t,:,1),x(t,:,2),'r')
    hold off
    xlim([1 dim/scale])
    ylim([1 dim/scale])
    pause(2)
end

%% testing area, ignore...

j = 1; t=0; next(j,:) = [6, 7, pi/2]; w(j) = 1;

for a=1:length(angle)
    for r=0:0.01:param(3)+1/scale
        new_x = next(j,1) + r*cos(next(j,3)+angle(a)); new_y = next(j,2) + r*sin(next(j,3)+angle(a));
        new_x = floor(new_x*scale); new_y = floor(new_y*scale);

        if next(j,1) < 0 || next(j,1) > dim/scale || next(j,2) > dim/scale || next(j,2) < 0
            w(j) = 0;
            break 
        end

        if new_x == 0 || new_y == 0
            w(j) = 0;
            break
        end

        if map(new_x, new_y) == 1
            w(j) = w(j)*10*prob_sensor_measurement(z(a),r,param);
            rr(a) = r;
            
            break
        end
    end
    if r > param(3)
        w(j) = w(j)*10*prob_sensor_measurement(z(a),r,param);
        rr(a) = param(3);
    end
    disp(w(j))
    
    
    theta = rem(angle(a) + next(j,3), 2*pi);
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
%     disp(true_distance)

    z(a) = gen_sensor_measurement(true_distance, param);

end
disp(w(j))
