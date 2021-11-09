function m = update_map(x)
% disp(x)
% the scaling of a pixel to actual length
scale = 10;

% create map (log likelihood) and inference parameter
dim = 150;
m = zeros(dim);
param = [0.05, 0.02, 5, 0.1, 0.1];

% at every step, send out beams in every direction
for angle = 0:0.01:2*pi
    % tune angle to be in (0,2pi)
    theta = rem(angle + x(3), 2*pi);

    % find the closest distance by searching through the beam

    % true_distance = 1000;
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

% get map into probabilty
m = 1-1./(1+exp(m));


end

