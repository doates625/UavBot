function cpp_mats()
%CPP_MATS() Print embedded matrices in C++ format
%   Author: Dan Oates (WPI Class of 2020)

% Title Printout
clc
fprintf('Embedded Matrices\n\n')
model = UAV.Models.Phys();

% D_bar Matrix
fprintf('D_bar Matrix:\n\n')
print_mat_cpp('D_bar', model.D_bar_ang);
fprintf('\n')

% M_lin Matrix
fprintf('M_lin Matrix:\n\n')
print_mat_cpp('M_lin', model.M_lin);
fprintf('\n')

end