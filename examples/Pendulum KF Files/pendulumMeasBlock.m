classdef pendulumMeasBlock < measurementblock

    methods
        function [H] = updateObservationMatrix(obj, x_, y_)
            H = eye(2);
        end
        function [R] = calcMeasurementCovarianceMatrix(obj)
            R = diag(obj.meas_sigmas.^2);
        end
        function [yhat, H] = update(obj, x_, y_, relinearize)
            if ~exist('relinearize', 'var'); relinearize = false; end
            if relinearize || all(isnan(obj.H),'all')
                obj.H = obj.updateObservationMatrix(x_, y_);
                H = obj.H;
                yhat = H*x_;
            else
                H = obj.H;
                yhat = H*x_;
            end
        end
    end

end

