function print_unit_vecs()
%PRINT_UNIT_VECS() Prints unit vectors in C++ format
%   Author: Dan Oates (WPI Class of 2020)

% Title Printout
clc
fprintf('Unit Vectors\n\n')

% Unit Vectors
names = {'x', 'y', 'z'};
I = eye(3);
for i = 1:3
    name = sprintf('%s_hat', names{i});
    print_vec_cpp(name, I(:,i));
    fprintf('\n')
end

end