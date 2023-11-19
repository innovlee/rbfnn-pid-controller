function rbfnn_mobile_robot_simulation()
    
    % simulation parameter
    simulation_time = 10; % Simulation time in seconds
    dt = 0.1; % Time step
    vx=10;
    vy=10;
    w=3;

    % Initial error
    error_prev_x = 0;
    error_prev_y = 0;
    error_prev_theta = 0;
    error_prev2_x = 0;
    error_prev2_y = 0;
    error_prev2_theta = 0;
    
    % initial state [x, y, theta]
    robot_state = [0; 0; 0];
    
    % desired state [x, y, theta]
    desired_state = [-1; -1.9; pi/8]; 

    % Initialize PID controllers and RBF Neural Networks for x, y, theta
    [pid_x, pid_y, pid_theta, rbfnn_x, rbfnn_y, rbfnn_theta] = pid_rbfnn_init();

    % Initialize the arrays to store data for plotting
    num_steps = simulation_time / dt;
    x_history = zeros(1, num_steps);
    y_history = zeros(1, num_steps);
    desired_x_history = zeros(1, num_steps);
    desired_y_history = zeros(1, num_steps);
    desired_theta_history = zeros(1, num_steps);
    actual_theta_history = zeros(1, num_steps);
    time_stamps = 0:dt:(simulation_time-dt);
 
   % Main simulation loop
    for t_idx = 1:num_steps
        t = (t_idx - 1) * dt;

        % Calculate x, y, theta error
        error_x = desired_state(1) - robot_state(1); 
        error_y = desired_state(2) - robot_state(2);
        error_theta = desired_state(3) - robot_state(3);
      
        % Update each axis
        [pid_x, ~] = rbfnn_update_x(rbfnn_x, error_x, pid_x, error_prev_x, error_prev2_x);
        [pid_y, ~] = rbfnn_update_y(rbfnn_y, error_y, pid_y, error_prev_y, error_prev2_y);
        [pid_theta, ~] = rbfnn_update_theta(rbfnn_theta, error_theta, pid_theta, error_prev_theta, error_prev2_theta);

        % PID control for x, y, and theta
        [pid_x, u_x] = pid_control(pid_x, error_x);
        [pid_y, u_y] = pid_control(pid_y, error_y);
        [pid_theta, u_theta] = pid_control(pid_theta, error_theta);

        
        % Apply the control signals to the robot and update its state
        robot_state = update_robot_state(robot_state, u_x, u_y, u_theta,vx,vy,w,dt);

        % Store data for ploting
        x_history(t_idx) = robot_state(1);
        y_history(t_idx) = robot_state(2);
        desired_x_history(t_idx) = desired_state(1);
        desired_y_history(t_idx) = desired_state(2);
        desired_theta_history(t_idx) = desired_state(3);
        actual_theta_history(t_idx) = robot_state(3);
        % desired_state

        % Calculate and store error data for plotting
        x_error = wrapToPi(desired_state(1) - robot_state(1)); %initial robot state [0,0,0]
        y_error = wrapToPi(desired_state(2) - robot_state(2));
        theta_error = wrapToPi(desired_state(3) - robot_state(3));
        x_error_history(t_idx) = abs(x_error);
        y_error_history(t_idx) = abs(y_error);
        theta_error_history(t_idx) = abs(theta_error);

        % Update error
       
        error_prev2_x = error_prev_x; % initial error_prev2, error_prev, error = 0
        error_prev_x = error_x;
    
        error_prev2_y = error_prev_y;
        error_prev_y = error_y;
      
        error_prev2_theta = error_prev_theta;
        error_prev_theta = error_theta; 
      
    end

    % Calculate and display error performance
    avg_x_error_percent = mean(x_error_history) / abs(desired_state(1)) * 100;
    max_x_error_percent = max(x_error_history) / abs(desired_state(1)) * 100;
    avg_y_error_percent = mean(y_error_history) / abs(desired_state(2)) * 100;
    max_y_error_percent = max(y_error_history) / abs(desired_state(2)) * 100;
    avg_theta_error_percent = mean(theta_error_history) / abs(wrapToPi(desired_state(3))) * 100;
    max_theta_error_percent = max(theta_error_history) / abs(wrapToPi(desired_state(3))) * 100;

    fprintf('average X Error Percentage: %.2f%%\n', avg_x_error_percent);
    fprintf('Max X Error Percentage: %.2f%%\n', max_x_error_percent);
    fprintf('average Y Error Percentage: %.2f%%\n', avg_y_error_percent);
    fprintf('Max Y Error Percentage: %.2f%%\n', max_y_error_percent);
    fprintf('average Theta Error Percentage: %.2f%%\n', avg_theta_error_percent);
    fprintf('max Theta Error Percentage: %.2f%%\n', max_theta_error_percent);

    % Plot the results
    plot_results(time_stamps, desired_theta_history, actual_theta_history, x_history, y_history, desired_x_history, desired_y_history);
end

% Define the PID control function
function [pid, u] = pid_control(pid, error)
    % Incremental PID control
    u_increment = pid.Kp * (error - pid.prev_error) + ...
                  pid.Ki * error + ...
                  pid.Kd * (error - 2*pid.prev_error + pid.prev_prev_error);

    % Update control signal
    u = pid.prev_u + u_increment;

    % Update Error
    pid.prev_prev_error = pid.prev_error;
    pid.prev_error = error;
    pid.prev_u = u; % Store the current control for next iteration
end

% Initial PID and rbfnn
function [pid_x, pid_y, pid_theta, rbfnn_x, rbfnn_y, rbfnn_theta] = pid_rbfnn_init()

    % Initiate Pid_x controller
    pid_x.Kp = 2;
    pid_x.Ki = 1;
    pid_x.Kd = 0.01;
    pid_x.prev_error = 0;
    pid_x.prev_prev_error = 0;
    pid_x.prev_u = 0;

      % Initiate Pid_y controller
    pid_y.Kp = 2;
    pid_y.Ki = 1;
    pid_y.Kd = 0.05;
    pid_y.prev_error = 0;
    pid_y.prev_prev_error = 0;
    pid_y.prev_u = 0;

     % Initiate Pid_theta controller
    pid_theta.Kp = 2;
    pid_theta.Ki = 1;
    pid_theta.Kd = 0.03;
    pid_theta.prev_error = 0;
    pid_theta.prev_prev_error = 0;
    pid_theta.prev_u = 0;

     % Initiate rbfnn_x controller
    rbfnn_x.centers = rand(4, 1);
    rbfnn_x.widths = rand(4, 1); 
    rbfnn_x.weights = rand(4, 3); %  3-4-3 rbfnn structure
    rbfnn_x.weights_prev = rbfnn_x.weights;
    rbfnn_x.weights_prev2 = rbfnn_x.weights;
    rbfnn_x.centers_prev = rbfnn_x.centers;
    rbfnn_x.centers_prev2 = rbfnn_x.centers;
    rbfnn_x.widths_prev = rbfnn_x.widths;
    rbfnn_x.widths_prev2 = rbfnn_x.widths;

        % Initiate rbfnn_y controller
    rbfnn_y.centers = rand(4, 1);
    rbfnn_y.widths = rand(4, 1);
    rbfnn_y.weights = rand(4, 3);
    rbfnn_y.weights_prev = rbfnn_y.weights;
    rbfnn_y.weights_prev2 = rbfnn_y.weights;
    rbfnn_y.centers_prev = rbfnn_y.centers;
    rbfnn_y.centers_prev2 = rbfnn_y.centers;
    rbfnn_y.widths_prev = rbfnn_y.widths;
    rbfnn_y.widths_prev2 = rbfnn_y.widths;

     % Initiate rbfnn_theta controller
    rbfnn_theta.centers = rand(4, 1);
    rbfnn_theta.widths = rand(4, 1);
    rbfnn_theta.weights = rand(4, 3);
    rbfnn_theta.weights_prev = rbfnn_theta.weights;
    rbfnn_theta.weights_prev2 = rbfnn_theta.weights;
    rbfnn_theta.centers_prev = rbfnn_theta.centers;
    rbfnn_theta.centers_prev2 = rbfnn_theta.centers;
    rbfnn_theta.widths_prev = rbfnn_theta.widths;
    rbfnn_theta.widths_prev2 = rbfnn_theta.widths;
end


function output = calculate_rbfnn_output(rbfnn, error)  %kp,ki,kd 
    
    num_neurons = length(rbfnn.centers); %4
    num_outputs = size(rbfnn.weights, 2);
    output = zeros(num_outputs, 1);

    for l = 1:num_outputs
        for j = 1:num_neurons
            % activate function : gausian
            h_j = exp(-(error - rbfnn.centers(j))^2 / (2 * rbfnn.widths(j)^2));
            % Calculate output
            output(l) = output(l) + rbfnn.weights(j, l) * h_j;
        end
    end
end

function [pid_x, rbfnn_x] = rbfnn_update_x(rbfnn_x, error_x, pid_x, error_prev_x, error_prev2_x)
   % Update x-axis RBFNN, PID
    eta = 0.1;  % Learning rate
    alpha = 0.01;  % coefficient

    
    for j = 1:length(rbfnn_x.centers)
        h_j = exp(-(error_x - rbfnn_x.centers(j))^2 / (2 * rbfnn_x.widths(j)^2));
        dy_du = sign(error_x); % ∂y/∂u 

        du_dol_sum = 0;

        % ∂u/∂o_l definition
        for l = 1:3 
            
            if l == 1
                du_dol = error_x - error_prev_x; % e(k) - e(k-1)
            elseif l == 2
                du_dol = error_x; % e(k)
            elseif l == 3
                du_dol = error_x - 2 * error_prev_x + error_prev2_x; % e(k) - 2e(k-1) + e(k-2)
            end

             du_dol_sum = du_dol_sum + du_dol;

               % update output layer weights 
            delta_w_jl = error_x * dy_du * du_dol * h_j;
            rbfnn_x.weights(j, l) = rbfnn_x.weights(j, l) + eta * delta_w_jl + alpha * (rbfnn_x.weights_prev(j, l) - rbfnn_x.weights_prev2(j, l));

  
            % hidden layer width and center update
            delta_c_ij = eta * error_x * dy_du * du_dol_sum * rbfnn_x.weights(j, l) * h_j * ((error_x - rbfnn_x.centers(j)) / rbfnn_x.widths(j)^2);
            rbfnn_x.centers(j) = rbfnn_x.centers(j) + delta_c_ij + alpha * (rbfnn_x.centers_prev(j) - rbfnn_x.centers_prev2(j));

            delta_sigma_j = eta * error_x * dy_du *du_dol_sum * rbfnn_x.weights(j, l) * h_j * ((error_x - rbfnn_x.centers(j))^2 / rbfnn_x.widths(j)^3);
            rbfnn_x.widths(j) = rbfnn_x.widths(j) + delta_sigma_j + alpha * (rbfnn_x.widths_prev(j) - rbfnn_x.widths_prev2(j));
        end
    end

   % Calculate Rbfnn and update kp,ki,kd
    rbfnn_output = calculate_rbfnn_output(rbfnn_x, error_x);
    pid_x.Kp = rbfnn_output(1);
    pid_x.Ki = rbfnn_output(2);
    pid_x.Kd = rbfnn_output(3);

    % Store ex weights , center, widths
    rbfnn_x.weights_prev2 = rbfnn_x.weights_prev;
    rbfnn_x.weights_prev = rbfnn_x.weights;
    rbfnn_x.centers_prev2 = rbfnn_x.centers_prev;
    rbfnn_x.centers_prev = rbfnn_x.centers;
    rbfnn_x.widths_prev2 = rbfnn_x.widths_prev;
    rbfnn_x.widths_prev = rbfnn_x.widths;
end


function [pid_y, rbfnn_y] = rbfnn_update_y(rbfnn_y, error_y, pid_y, error_prev_y, error_prev2_y)
    % Update y-axis RBFNN, PID 
    eta = 0.1;  % Learning rate
    alpha = 0.01;  % coefficient

    
    for j = 1:length(rbfnn_y.centers)
        h_j = exp(-(error_y - rbfnn_y.centers(j))^2 / (2 * rbfnn_y.widths(j)^2));
        dy_du = sign(error_y); % ∂y/∂u 

        du_dol_sum = 0;

         % ∂u/∂o_l definition
        for l = 1:3 
            
            if l == 1
                du_dol = error_y - error_prev_y; % e(k) - e(k-1)
            elseif l == 2
                du_dol = error_y; % e(k)
            elseif l == 3
                du_dol = error_y - 2 * error_prev_y + error_prev2_y; % e(k) - 2e(k-1) + e(k-2)
            end

             du_dol_sum = du_dol_sum + du_dol;

            % Update output layer weights
            delta_w_jl = error_y * dy_du * du_dol * h_j;
            rbfnn_y.weights(j, l) = rbfnn_y.weights(j, l) + eta * delta_w_jl + alpha * (rbfnn_y.weights_prev(j, l) - rbfnn_y.weights_prev2(j, l));

  
            % Update hidden layer center and widths
            delta_c_ij = eta * error_y * dy_du * du_dol_sum * rbfnn_y.weights(j, l) * h_j * ((error_y - rbfnn_y.centers(j)) / rbfnn_y.widths(j)^2);
            rbfnn_y.centers(j) = rbfnn_y.centers(j) + delta_c_ij + alpha * (rbfnn_y.centers_prev(j) - rbfnn_y.centers_prev2(j));

            delta_sigma_j = eta * error_y * dy_du *du_dol_sum * rbfnn_y.weights(j, l) * h_j * ((error_y - rbfnn_y.centers(j))^2 / rbfnn_y.widths(j)^3);
            rbfnn_y.widths(j) = rbfnn_y.widths(j) + delta_sigma_j + alpha * (rbfnn_y.widths_prev(j) - rbfnn_y.widths_prev2(j));
        end
    end

    % Calculate Rbfnn and update kp,ki,kd
    rbfnn_output = calculate_rbfnn_output(rbfnn_y, error_y);
    pid_y.Kp = rbfnn_output(1);
    pid_y.Ki = rbfnn_output(2);
    pid_y.Kd = rbfnn_output(3);

    % Store ex weights , center, widths
    rbfnn_y.weights_prev2 = rbfnn_y.weights_prev;
    rbfnn_y.weights_prev = rbfnn_y.weights;
    rbfnn_y.centers_prev2 = rbfnn_y.centers_prev;
    rbfnn_y.centers_prev = rbfnn_y.centers;
    rbfnn_y.widths_prev2 = rbfnn_y.widths_prev;
    rbfnn_y.widths_prev = rbfnn_y.widths;
end

function [pid_theta, rbfnn_theta] = rbfnn_update_theta(rbfnn_theta, error_theta, pid_theta, error_prev_theta, error_prev2_theta)
    % Update theta RBFNN, PID 
    eta = 0.1;  % learning rate
    alpha = 0.01;  % coefficient

   
    for j = 1:length(rbfnn_theta.centers)
        h_j = exp(-(error_theta - rbfnn_theta.centers(j))^2 / (2 * rbfnn_theta.widths(j)^2));
        dy_du = sign(error_theta); % ∂y/∂u 

        du_dol_sum = 0;

       % ∂u/∂o_l definition
        for l = 1:3 % output layer neurons 
           
            if l == 1
                du_dol = error_theta - error_prev_theta; % e(k) - e(k-1)
            elseif l == 2
                du_dol = error_theta; % e(k)
            elseif l == 3
                du_dol = error_theta - 2 * error_prev_theta + error_prev2_theta; % e(k) - 2e(k-1) + e(k-2)
            end

             du_dol_sum = du_dol_sum + du_dol;

            % update output layer weights 
            delta_w_jl = error_theta * dy_du * du_dol * h_j;
            rbfnn_theta.weights(j, l) = rbfnn_theta.weights(j, l) + eta * delta_w_jl + alpha * (rbfnn_theta.weights_prev(j, l) - rbfnn_theta.weights_prev2(j, l));

  
            % hidden layer width and center update
            delta_c_ij = eta * error_theta * dy_du * du_dol_sum * rbfnn_theta.weights(j, l) * h_j * ((error_theta - rbfnn_theta.centers(j)) / rbfnn_theta.widths(j)^2);
            rbfnn_theta.centers(j) = rbfnn_theta.centers(j) + delta_c_ij + alpha * (rbfnn_theta.centers_prev(j) - rbfnn_theta.centers_prev2(j));

            delta_sigma_j = eta * error_theta * dy_du *du_dol_sum * rbfnn_theta.weights(j, l) * h_j * ((error_theta - rbfnn_theta.centers(j))^2 / rbfnn_theta.widths(j)^3);
            rbfnn_theta.widths(j) = rbfnn_theta.widths(j) + delta_sigma_j + alpha * (rbfnn_theta.widths_prev(j) - rbfnn_theta.widths_prev2(j));
        end
    end

    % Calculate Rbfnn and update kp,ki,kd
    rbfnn_output = calculate_rbfnn_output(rbfnn_theta, error_theta);
    pid_theta.Kp = rbfnn_output(1);
    pid_theta.Ki = rbfnn_output(2);
    pid_theta.Kd = rbfnn_output(3);

   % Store ex weights , center, widths
    rbfnn_theta.weights_prev2 = rbfnn_theta.weights_prev;
    rbfnn_theta.weights_prev = rbfnn_theta.weights;
    rbfnn_theta.centers_prev2 = rbfnn_theta.centers_prev;
    rbfnn_theta.centers_prev = rbfnn_theta.centers;
    rbfnn_theta.widths_prev2 = rbfnn_theta.widths_prev;
    rbfnn_theta.widths_prev = rbfnn_theta.widths;
end

% update robot state 
function robot_state = update_robot_state(state, u_x, u_y, u_theta,vx,vy,w,dt)
    % present state
    x= state(1);
    y= state(2);
    theta = state(3);
    

    A =[cos(theta) sin(theta) 0; -sin(theta) cos(theta) 0; 0 0 1];
    B= [vx; vy; w];
    delta_state = A * B * dt;
    
    % calculate next position and angle
    new_x = x + delta_state(1) + (u_x)*dt;
    new_y = y + delta_state(2) + (u_y)*dt;
    new_theta = theta + delta_state(3) + (u_theta)*dt;
   
    % Update new data
    robot_state = [new_x; new_y; new_theta];
  
end

% Plotting function
function plot_results(time_stamps, desired_theta_history, actual_theta_history, x_history, y_history, desired_x_history, desired_y_history);

    % Plot for x positions
    figure;
    subplot(2,1,1);
    plot(time_stamps, x_history, 'b', time_stamps, desired_x_history, 'r--');
    title('X Position over Time');
    xlabel('Time (s)');
    ylabel('X Position');
    legend('Actual X', 'Desired X');
    grid on;

    % Plot for y positions
    subplot(2,1,2);
    plot(time_stamps, y_history, 'b', time_stamps, desired_y_history, 'r--');
    title('Y Position over Time');
    xlabel('Time (s)');
    ylabel('Y Position');
    legend('Actual Y', 'Desired Y');
    grid on;


    % Plot for Direction Angle
    figure;
    plot(time_stamps, desired_theta_history, 'r', time_stamps, actual_theta_history, 'b');
    xlabel('Time (s)');
    ylabel('Theta (radians)');
    title('Control Plot for Direction Angle');
    legend('Desired Theta', 'Actual Theta');
    grid on;

end



