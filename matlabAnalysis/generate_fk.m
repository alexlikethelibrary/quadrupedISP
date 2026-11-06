%% Step 1: Define leg lengths
l_body = 100; % Length from center of body to outer casing in mm
l_0 = 22.6; % Distance between outer casing and hip axis of rotation in mm
l_c = 44.5; % Length of coxa (distance between hip aor and knee aor) in mm
l_f = 70; % Length of femur (distance between knee aor and ankle aor) in mm
l_t = 86.05 + 17.5; % Length of tibia (distance between ankle aor and foot) in mm

%% Step 2: Define symbolic DH table for one robot leg

syms theta_h theta_k theta_a 
% theta_h is the angle of the hip joint
% theta_k is the angle of the knee joint
% theta_a is the angle of the ankle joint

% One leg's full DH Table
dhTable = [0,              0,   l_0 + (0.5*l_body),    0;
           theta_h,        0,   l_c,                 -90;
           theta_k,        0,   l_f,                   0;
           theta_a + 90,   0,   l_t,                   0;];

%% Step 3: Pass symbolic DH table into dh2fk to get symbolic FK matrix

fk= simplify(dh2fk(dhTable));

disp(fk);

%% Step 4: Feed symbolic FK matrix into 'matlabFunction' to turn it
% into a floating point precision function that runs fast.

fkHandle = matlabFunction(fk, 'file','fkHandle.m', "Vars", [theta_h, theta_k, theta_a]);

%% Step 5: plug in all possible values of theta to determine usable workspace
hipRange   = linspace(-70, 70, 50);   
kneeRange  = linspace(-90, 45, 50);
ankleRange = linspace(0, 90, 50);
[H, K, A] = ndgrid(hipRange, kneeRange, ankleRange);

vars = [theta_h, theta_k, theta_a];
xf = matlabFunction(fk(1,4), 'Vars', vars);
yf = matlabFunction(fk(2,4), 'Vars', vars);
zf = matlabFunction(fk(3,4), 'Vars', vars);

X = xf(H,K,A) + 0*H;   % + 0*H guards against a scalar result
Y = yf(H,K,A) + 0*H;
Z = zf(H,K,A) + 0*H;

%% Step 6: Visualize the reachable workspace
figure;
scatter3(X(:), Y(:), Z(:), 8, Z(:), 'filled');
axis equal;
grid on;
xlabel('X (mm)'); ylabel('Y (mm)'); zlabel('Z (mm)');
title('Leg Workspace');

%% Helper Function Definitions

function ht = dh2mat(dh_row)
% DH2MAT Gives the transformation matrix for a given row of a DH table
% Inputs:
%    dh_row: a [1x4] vector containing representing a single row of a DH
%            table
% Outputs: 
%    ht: a [4x4] matrix representing the transformation defined by the DH
%        row

    if (numel(dh_row) ~= 4)
        error('dh2mat:invalidInput', 'Input must be a 1x4 matrix');
    end
    
    %dh_row is [theta_i    d_i    r_i    alpha_i]
    theta_i = dh_row(1);
    d_i = dh_row(2); 
    r_i = dh_row(3);
    alpha_i = dh_row(4);
    
    %standard dh convention rot z * trans Z * trans X * rot X
    ht = [cosd(theta_i),    -sind(theta_i)*cosd(alpha_i),   sind(theta_i)*sind(alpha_i),   r_i*cosd(theta_i);
          sind(theta_i),    cosd(theta_i)*cosd(alpha_i),  -cosd(theta_i)*sind(alpha_i),  r_i*sind(theta_i);
               0,                 sind(alpha_i),               cosd(alpha_i),                   d_i;
               0,                       0,                           0,                          1        ];
end

function ht = dh2fk(dh_tab)
% DH2FK Calculates FK from a DH table
% Inputs:
%    dh_tab: a [nx4] matrix representing a DH table
% Outputs: 
%    ht: a [4x4] matrix representing the transformation defined by the DH
%        table

    % corresponding to one row of the DH table
    if (size(dh_tab, 2) ~= 4)
        error('dh2fk:invalidInput', 'Input must be an nx4 matrix');
    end

    numRows = size(dh_tab, 1);
    %start form base frame
    ht = eye(4);

    %progressively transform frame to frame
    for j = 1:numRows
        frameTransfer = dh2mat(dh_tab(j, :));
        ht = ht *frameTransfer;
    end

end