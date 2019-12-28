function euler_matrix()
%EULER_MATRIX() Prints symbolic expression for Z-Y-X euler rotation matrix

% Symbolic Euler Rotations
clear, clc
disp('Symbolic Euler Rotation')

% Z-Axis Rotation
disp('Z-Axis (Rz)')
syms cz sz
Rz = [...
    cz, -sz, 0; ...
    sz,  cz, 0; ...
     0,   0, 1];
disp(Rz)

% Y-Axis Rotation
disp('Y-Axis (Ry)')
syms cy sy
Ry = [...
     cy, 0, sy; ...
      0, 1,  0; ...
    -sy, 0, cy];
disp(Ry)

% X-Axis Rotation
disp('X-Axis (Rx)')
syms cx sx
Rx = [...
    1,  0,   0; ...
    0, cx, -sx; ...
    0, sx,  cx];
disp(Rx)

% Full Rotation
R = Rz * Ry * Rx;
disp('Combined (Rz * Ry * Rx)')
disp(R)

end