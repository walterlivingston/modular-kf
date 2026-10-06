classdef rejectAllCovBlock < covarianceblock
    %REJECTALLCOVBLOCK Covariance block that rejects every measurement
    % Used by the unit tests to exercise the rejection path.

    methods
        function [S, reject] = calcInnovationCovarianceMatrix(obj, filter, H, R)
            S = H*filter.P*H' + R;
            obj.S = S;
            reject = true;
        end
    end
end
