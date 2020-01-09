classdef Enum < uint8
    %ENUM UAV state enumeration
    %   Author: Dan Oates (WPI Class of 2020)
    
    enumeration
        Enabled (0)     % Flight enabled
        Disabled (1)    % Flight disabled
        Failed (2)      % UAV crashed
    end
end