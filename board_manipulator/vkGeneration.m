% Joint Lengths and DH Parameters for Terrawarden Mansplain Robot
% Meters & Radians
L1 = 0.20375 - 0.02;
L2 = hypot(0.23746, 0.017);
L3 = 0.25;
L4 = 0.02;
jointOffset = atan(17 / 237.46);

syms tht1 tht2 tht3;
thts = [tht1, tht2, tht3]; % symbolic joint variables
dh_table_const = [
    -tht1, -L1, 0, pi/2;
    tht2-pi/2 + jointOffset, 0, L2, 0;
    tht3+pi/2 - jointOffset, -L4, L3, -pi/2
];

fk = eye(4); % Initialize the forward kinematics matrix as identity

fk = fk * dh2mat(dh_table_const(1,:)); % T1
fk = fk * dh2mat(dh_table_const(2,:)); % T2
fk = fk * dh2mat(dh_table_const(3,:)); % T3
% The final fk matrix represents the end-effector position and orientation

%jakubian = zeros(6,3); % Initialize the Jacobian matrix

jakubian(1:3, 1) = diff(fk(1:3, 4),tht1); % Linear velocity part (position)
jakubian(1:3, 2) = diff(fk(1:3, 4),tht2); % Linear velocity part (position)
jakubian(1:3, 3) = diff(fk(1:3, 4),tht3); % Linear velocity part (position)

matlabFunction(jakubian, 'Vars', {tht1, tht2, tht3}, 'File', 'vk.m', 'Optimize', true);

