classdef fullMeasBlock < measurementblock
    %FULLMEASBLOCK Measurement block that observes every state directly
    % Used by the unit tests for multi-element measurements.

    methods
        function [H] = updateObservationMatrix(obj, x_, y_)
            H = eye(numel(x_));
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
