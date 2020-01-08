function arg = default_arg(name)
%DEFAULT_ARG Get default function arg by name
%   
%   arg = DEFAULT_ARG('model') UAV model [UAV.Models.Model]
%   arg = DEFAULT_ARG('phys_model') UAV physical model [UAV.Models.Phys]
%   arg = DEFAULT_ARG('I_xx') Inertia x-axis [kg*m^2]
%   arg = DEFAULT_ARG('I_yy') Inertia y-axis [kg*m^2]
%   arg = DEFAULT_ARG('I_zz') Inertia z-axis [kg*m^2]
%   arg = DEFAULT_ARG('r_x') Moment arm x-axis [m]
%   arg = DEFAULT_ARG('r_y') Moment arm y-axis [m]
%   arg = DEFAULT_ARG('r_z') Moment arm z-axis [m]
%   arg = DEFAULT_ARG('mass') Total mass [kg]
%   arg = DEFAULT_ARG('f_min') Min prop force [N]
%   arg = DEFAULT_ARG('f_max') Max prop force [N]
%   arg = DEFAULT_ARG('ctrl_model') UAV control model [UAV.Models.Ctrl]
%   arg = DEFAULT_ARG('s_qx') Quaternion x-axis pole [s^-1]
%   arg = DEFAULT_ARG('s_qy') Quaternion y-axis pole [s^-1]
%   arg = DEFAULT_ARG('s_qz') Quaternion z-axis pole [s^-1]
%   arg = DEFAULT_ARG('s_az') Acceleration z-axis pole [s^-1]
%   arg = DEFAULT_ARG('fr_min') Min prop thrust ratio [N/N]
%   arg = DEFAULT_ARG('fr_max') Max prop thrust ratio [N/N]
%   arg = DEFAULT_ARG('f_sim') Simulation frequency [Hz]
%   arg = DEFAULT_ARG('bt_name') Bluetooth device name [char]
%   arg = DEFAULT_ARG('sim_port') Simulation serial port name [char]
%   arg = DEFAULT_ARG('acc_max') Max acceleration cmd [m/s^2]
%   arg = DEFAULT_ARG('wz_max') Max heading cmd rate [rad/s]

switch name
    % Full model
    case 'model', arg = UAV.Models.Model();
    
    % Physical model
    case 'phys_model', arg = UAV.Models.Phys();
    case 'I_xx', arg = 1.15e-03;
    case 'I_yy', arg = 1.32e-03;
    case 'I_zz', arg = 2.24e-03;
    case 'r_x', arg = 9.30e-02;
    case 'r_y', arg = 9.30e-02;
    case 'r_z', arg = 5.50e-02;
    case 'mass', arg = 0.546;
    case 'f_min', arg = 0.00;
    case 'f_max', arg = 2.46;
        
    % Control model
    case 'ctrl_model', arg = UAV.Models.Ctrl();
    case 's_qx', arg = -3.0;
    case 's_qy', arg = -3.0;
    case 's_qz', arg = -3.0;
    case 's_az', arg = -3.0;
    case 'fr_min', arg = 0.1;
    case 'fr_max', arg = 0.9;
    
    % Simulation
    case 'f_sim', arg = 50.0;
        
    % Communication
    case 'remote', arg = UAV.Interfaces.Remote();
    case 'bt_name', arg = 'UavBot';
    case 'sim_port', arg = 'COM17';
        
    % Piloting
    case 'acc_max', arg = 1.0;
    case 'wz_max', arg = pi/4;
        
    % Invalid name
    otherwise, error('Invalid name: %s', name);
end

end