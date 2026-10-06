classdef posMeasBlock < measurementblock
    %POSMEASBLOCK Position measurement block used by the unit tests
    % Deliberately returns H from update() without storing it in obj.H, so
    % the tests catch any code that reads H from the block instead of
    % using the H the filter was given.

    methods
        function [H] = updateObservationMatrix(obj, x_, y_)
            H = [1 0];
        end
        function [R] = calcMeasurementCovarianceMatrix(obj)
            R = diag(obj.meas_sigmas.^2);
        end
        function [yhat, H] = update(obj, x_, y_, relinearize)
            H = obj.updateObservationMatrix(x_, y_);
            yhat = H*x_;
        end
    end
end
