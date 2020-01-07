function arm_mats()
%ARM_MATS() Print symbolic moment arm matrices
%   Author: Dan Oates (WPI Class of 2020)

% Title Printout
clc
fprintf('Symbolic Moment Arm Matrices\n\n')

% Moment Arm
fprintf('D Matrix\n')
syms r_x r_y r_z
D = [...
    +r_x, -r_x, +r_x, -r_x; ...
    -r_y, -r_y, +r_y, +r_y; ...
    +r_z, -r_z, -r_z, +r_z; ...
       1,    1,    1,    1];
disp(D)

% Inverse Moment Arm
fprintf('D-bar Matrix\n')
D_bar = inv(D);
D_bar = D_bar(1:4, 1:3);
disp(D_bar)

end