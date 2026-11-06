% Create a robot object
robot = Robot();

matrixSize = 500;
% Define three joint positions to form the vertices of your triangle
% Make sure theta_1 = 0 for all of these configurations!

pos1 = [0, 0, 0, 0];
pos2 = [0, 10, 60, -60];
pos3 = [0, -50, 40, -90];
posArray = [pos2; pos3; pos1];

jointPos = zeros(matrixSize, 4);
times = zeros (matrixSize, 1);


% Move your robot to the first vertex
interpolate_time = 2000;

robot.interpolate_jp(pos1,interpolate_time);
pause(interpolate_time/1000);
% Create a for loop that sends your robot to each vertex, recording the
% joint-space position the entire time
j = 1; %position idex
for i=1:3
    robot.interpolate_jp(posArray(i,:),interpolate_time);
    %pause(interpolate_time/1000);
    tic;  % Start timer
    while toc < interpolate_time/1000
        % Read current joint positions (not velocities though!)
        curr = measure_js(robot, true, false);
        % Store the positions and timesetamps in an array
        % Hint: use 'toc' to get the current elapsed time since tic
        jointPos(j, :) = curr(1, :); %insert measurements into positions
        times(j, :) = toc; %insert time data
        j= j + 1; %increment index
    end
    % See lab_demo.m from lab 1 for inspiration on how to move your robot
    % and collect data
end

cleanPos = jointPos(1:j-1, :);
cleanTimes = times(1:j-1, :);

% Loop through all collected data and convert it from joint-space to task
% space using your fk3001 function
xyzPos = zeros(j, 3);

for k = 1:j-1
    transMatrix = robot.fk3001(cleanPos(k, :));
    xyzPos(k, :) = [transMatrix(1,4) transMatrix(2,4) transMatrix(3,4)];  
end
% Plot the trajectory of your robot in x-y-z space using scatter3
% https://www.mathworks.com/help/matlab/ref/scatter3.html
figure;
scatter3(xyzPos(:,1), xyzPos(:, 2), xyzPos(:, 3));
axis equal 

% Plot the trajectory of your robot in theta2-theta3-theta-4 space using
% scatter3
figure;
scatter3(cleanPos(:,2), cleanPos(:, 3), cleanPos(:, 4));
axis equal 