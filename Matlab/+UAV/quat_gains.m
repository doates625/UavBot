function [k_p, k_i, k_d] = quat_gains(inr, rad, f_max, poles)
%[k_p, k_i, k_d] = QUAT_GAINS(inr, rad, f_max, poles)
%   Make quaternion PID gains
%   Inputs:
%       inr = Moment of inertia [kg*m^2]
%       rad = Moment arm radius [m]
%       f_max = Max prop force [N]
%       poles = Three poles [s1, s2, s3]
%   Outputs:
%       k_p = P-gain [thr/rad]
%       k_i = I-gain [thr/(rad*s)]
%       k_d = D-gain [thr/(rad/s)]

A = zeros(3, 3);
A(1, 2) = 1;
A(2, 3) = 1;
B = zeros(3, 1);
H = 2 * f_max * rad / inr;
B(3) = H;
K = place(A, B, poles);
k_i = K(1);
k_p = K(2);
k_d = K(3);

end