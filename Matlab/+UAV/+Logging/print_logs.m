function print_logs()
    %PRINT_LOGS Prints UAV flight log names and comments
    %   Author: Dan Oates (WPI Class of 2020)
    
    % Initial printout
    clc
    fprintf('UAV Flight Logs\n\n')
    
    % Process files
    path = UAV.Logging.Log.log_path;
    files = dir([path, '*.mat']);
    for f = 1:length(files)
        
        % Print file name
        name = files(f).name;
        name = name(1:end-4);
        fprintf('%s:\n', name);
        
        % Print comments
        log = UAV.Logging.Log(name);
        for c = 1:length(log.comments)
            fprintf('\t- %s\n', log.comments{c});
        end
        
        % Print gap
        fprintf('\n')
    end
end