classdef basicCovBlock < covarianceblock
    % 
    %
    % Author: Walter Livingston
    
    properties
    end

    methods
        function [S, reject] = calcInnovationCovarianceMatrix(obj, filter, H, R)
            z = filter.z;
            S = H*filter.P*H' + R;
            obj.S = S;
            obj.r = sqrt(z'*(S\z));

            reject = false;
        end
    end
end

