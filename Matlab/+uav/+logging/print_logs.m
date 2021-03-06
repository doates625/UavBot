function print_logs()
    %PRINT_LOGS Prints UAV flight log names and comments
    %   
    %   Author: Dan Oates (WPI Class of 2020)
    
    % Imports
    import('uav.logging.Log');
    
    % Initial printout
    clc
    fprintf('UAV Flight Logs\n\n')
    
    % Process files
    path = Log.log_path;
    files = dir([path, '*.mat']);
    for f = 1:length(files)
        
        % Print file name
        name = files(f).name;
        name = name(1:end-4);
        fprintf('%s:\n', name);
        
        % Print comments
        log = Log(name);
        n_c = length(log.comments);
        if n_c > 0
            for c = 1:n_c
                fprintf('\t- %s\n', log.comments{c});
            end
        else
            fprintf('\t(No comments)\n');
        end
        
        % Print gap
        fprintf('\n')
    end
end