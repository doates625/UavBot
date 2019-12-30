function print_mass_mats(model)
%PRINT_MASS_MATS(model) Prints values of mass matrices in C++ format
%   model = UAV model [UavModel]
%   Author: Dan Oates (WPI Class of 2020)

% Default arg
if nargin < 1
    model = UavModel();
end

% Title Printout
clc
fprintf('UAV Mass Matrices\n\n')

% Angular Mass Matrix
fprintf('Angular Matrix:\n\n')
print_mat_cpp('M_alp', model.M_alp)
fprintf('\n')

% Linear Mass Matrix
fprintf('Linear Matrix:\n\n')
print_mat_cpp('M_acc', model.M_acc)
fprintf('\n')

end