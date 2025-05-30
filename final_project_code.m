% Multi-Robot System Simulation in 2D Space with Debris-Driven Potential Functions

% Parameters
sensor_range = 10;    % Sensor range for each robot
communication_range = 15; % Communication range for each robot
N = 10;              % Number of robots
D = 10;              % Number of expected_debris
sigma_x = 3.0;       % Spread of potential function in x-direction
sigma_y = 3.0;       % Spread of potential function in y-direction
space_size = 50;     % Size of the 2D space
num_steps = 10000;    % Number of simulation steps
dt = 0.01;           % Time step for integration
tolerance = 0.5;     % Distance threshold to consider a robot near the maximum
exploration_steps = 500; % Number of steps to remain at maximum before exploring
exploration_duration = 1000; % Steps to remain in exploration before switching back to attraction
velocity_norm = 1.0; % Desired constant velocity magnitude
noise_strength = 0.25; % Strength of noise added to robot motion
occupancy_grid = ones(space_size, space_size, N); % Initialize occupancy grids for each robot

% Initialize robot and expected_debris positions
robots = rand(N, 2) * space_size; % Random positions in 50x50 space
expected_debris = round(rand(D, 2) * space_size);
max_displacement = 15;
actual_debris = round(expected_debris + (rand(D, 2) - 0.5) * max_displacement); % Slightly displaced debris points % Random positions in 50x50 space
actual_debris(:, 1) = max(0, min(50, actual_debris(:, 1))); % Constrain to domain
actual_debris(:, 2) = max(0, min(50, actual_debris(:, 2)));

% Create a figure for visualization
communication_circles = gobjects(1, N); % Store communication range circles for each robot
sensor_circles = gobjects(1, N);
f1 = figure;
movegui(f1,[20 100]);
hold on;
axis([0 space_size 0 space_size]);
plot(expected_debris(:,1), expected_debris(:,2), 'rx', 'MarkerSize', 8, 'LineWidth', 2); % Plot expected_debris
for i = 1:N
    communication_circles(i) = rectangle('Position', [robots(i,1)-communication_range/2, robots(i,2)-communication_range/2, communication_range, communication_range], 'Curvature', [1, 1], 'EdgeColor', 'cyan', 'LineStyle', '--');
    sensor_circles(i) = rectangle('Position', [robots(i,1)-sensor_range/2, robots(i,2)-sensor_range/2, sensor_range, sensor_range], 'Curvature', [1, 1], 'EdgeColor', 'magenta', 'LineStyle', '--');
end
plot(actual_debris(:,1), actual_debris(:,2), 'bx', 'MarkerSize', 8, 'LineWidth', 2); % Plot actual_debris
robots_plot = plot(robots(:,1), robots(:,2), 'bo', 'MarkerSize', 6, 'LineWidth', 1.5);
trajectories = gobjects(1, N); % Store trajectory plots for each robot
for i = 1:N
    trajectories(i) = plot(robots(i,1), robots(i,2), '-', 'LineWidth', 0.5); % Initialize trajectory plots
end
title('Multi-Robot System with Debris Potential');
xlabel('X'); ylabel('Y');
% legend("Expected Debris", "Actual Debris", "Robot Pos");
grid on;

% Visualize potential functions as a surface
[x, y] = meshgrid(linspace(0, space_size, 100), linspace(0, space_size, 100));
potential_surface = zeros(size(x));
for j = 1:D
    potential_surface = potential_surface + exp(-((x - expected_debris(j,1)).^2 / (2 * sigma_x^2) + (y - expected_debris(j,2)).^2 / (2 * sigma_y^2)));
end
f2=figure;
movegui(f2,[650 100]);
surf(x, y, potential_surface, 'EdgeColor', 'none');
colormap jet;
title('Potential Function Surface');
xlabel('X'); ylabel('Y'); zlabel('Potential');
grid on;
view(3);

% Simulation loop
attraction_phase = true(1, N); % Each robot starts in attraction phase
attraction_timer = zeros(1, N); % Timer for each robot to count steps near maximum
exploration_timer = zeros(1, N); % Timer for each robot to count steps in exploration

trajectories_data = cell(1, N); % Store trajectory data for each robot
for i = 1:N
    trajectories_data{i} = robots(i, :); % Initialize trajectory with initial position
end

for step = 1:num_steps
    % disp("step: ");disp(step);
    % Compute gradients of potential functions for each robot
    gradients = zeros(N, 2); % Gradient for each robot
    potentials = zeros(N, 1); % Potential values for each robot

    for i = 1:N
        for j = 1:D
            dx = robots(i,1) - expected_debris(j,1);
            dy = robots(i,2) - expected_debris(j,2);
            % Gradient of potential function
            grad_x = -exp(-((dx^2)/(2*sigma_x^2) + (dy^2)/(2*sigma_y^2))) * (dx / sigma_x^2);
            grad_y = -exp(-((dx^2)/(2*sigma_x^2) + (dy^2)/(2*sigma_y^2))) * (dy / sigma_y^2);
            gradients(i, :) = gradients(i, :) + [grad_x, grad_y];
            % Accumulate potential values
            potentials(i) = potentials(i) + exp(-((dx^2)/(2*sigma_x^2) + (dy^2)/(2*sigma_y^2)));
            distance_to_debris = norm(robots(i,:) - actual_debris(j,:));
            if distance_to_debris <= sensor_range/2
                debris_x = actual_debris(j,1);
                debris_y = actual_debris(j,2);
                occupancy_grid(debris_x, debris_y, i) = 0;
            end
        end
    end

    % Normalize velocities to a constant value
    for i = 1:N
        grad_norm = norm(gradients(i, :));
        if grad_norm > 0
            gradients(i, :) = (gradients(i, :) / grad_norm) * velocity_norm;
        end
    end

    % Perform occupancy grid updates for all robots
    updated_grid = occupancy_grid; % Create a temporary grid to avoid overwriting during updates
    for i = 1:N
        for j = 1:N
            if i ~= j && norm(robots(i,:) - robots(j,:)) <= communication_range
                % Update occupancy grid for robot i using robot j's grid
                updated_grid(:,:,i) = updated_grid(:,:,i) .* sqrt(occupancy_grid(:,:,j));
            end
        end
    end
    occupancy_grid = updated_grid; % Apply the updates after all computations

    for i = 1:N
        if attraction_phase(i)
            % Move towards the maxima
            robots(i, :) = robots(i, :) + dt * gradients(i, :);

            % Check if robot is close to the maximum
            if abs(potentials(i) - max(potentials)) < tolerance
                attraction_timer(i) = attraction_timer(i) + 1;
            else
                attraction_timer(i) = 0; % Reset timer if robot moves away
            end

            % Switch to exploration phase after waiting
            if attraction_timer(i) >= exploration_steps
                attraction_phase(i) = false;
                exploration_timer(i) = 0; % Reset exploration timer
            end

        else
            % Exploration phase: move away from the maxima by reversing the gradient and adding noise
            noise = noise_strength * (rand(1, 2) - 0.5); % Random noise in range [-0.5, 0.5]
            robots(i, :) = robots(i, :) - dt * gradients(i, :) + noise;

            % Increment exploration timer
            exploration_timer(i) = exploration_timer(i) + 1;

            % Switch back to attraction phase after exploration duration
            if exploration_timer(i) >= exploration_duration
                attraction_phase(i) = true;
                attraction_timer(i) = 0; % Reset attraction timer
            end
        end

        % Keep robots within bounds of the space
        robots(i, :) = max(0, min(space_size, robots(i, :)));

        % Update trajectory data
        trajectories_data{i} = [trajectories_data{i}; robots(i, :)];
    end

    % Update visualization
    for i = 1:N
        set(communication_circles(i), 'Position', [robots(i,1)-communication_range/2, robots(i,2)-communication_range/2, communication_range, communication_range]);
        set(sensor_circles(i), 'Position', [robots(i,1)-sensor_range/2, robots(i,2)-sensor_range/2, sensor_range, sensor_range]);
    end
    set(robots_plot, 'XData', robots(:,1), 'YData', robots(:,2));
    for i = 1:N
        set(trajectories(i), 'XData', trajectories_data{i}(:,1), 'YData', trajectories_data{i}(:,2));
    end
    drawnow;

end

for v=1:N
    figure('name', "robot"+num2str(v));
    map1 = squeeze(occupancy_grid(:,:,v));
    map1 = rot90(map1,1);
    show(binaryOccupancyMap(map1));
end