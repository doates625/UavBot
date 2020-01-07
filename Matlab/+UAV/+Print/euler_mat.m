function euler_mat()
%EULER_MAT() Print symbolic Z-Y-X euler rotation matrix
%   Author: Dan Oates (WPI Class of 2020)

% Title Printout
clc
fprintf('Symbolic Euler Rotation Matrix\n\n')

% Z-Axis Rotation
fprintf('Z-axis (Rz)\n')
syms cz sz
Rz = [...
    cz, -sz, 0; ...
    sz,  cz, 0; ...
     0,   0, 1];
disp(Rz)

% Y-Axis Rotation
fprintf('Y-axis (Ry)\n')
syms cy sy
Ry = [...
     cy, 0, sy; ...
      0, 1,  0; ...
    -sy, 0, cy];
disp(Ry)

% X-Axis Rotation
fprintf('X-axis (Rx)\n')
syms cx sx
Rx = [...
    1,  0,   0; ...
    0, cx, -sx; ...
    0, sx,  cx];
disp(Rx)

% Full Rotation
R = Rz * Ry * Rx;
fprintf('Combined (Rz * Ry * Rx)\n')
disp(R)

end