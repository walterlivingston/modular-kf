classdef cvStateBlock < stateblock
    %CVSTATEBLOCK Constant velocity state block used by the unit tests
    % States are [position; velocity], with white acceleration noise and
    % an optional acceleration input u.

    methods
        function [obj] = cvStateBlock(accel_sigma)
            obj@stateblock(2, accel_sigma);
        end

        function [F] = updateStateTransitionMatrix(obj, x_)
            F = [0 1;
                 0 0];
        end
        function [x] = updateState(obj, x_, dt, u)
            if ~exist('u', 'var'); u = 0; end
            x = [x_(1) + x_(2)*dt + u*dt^2/2;
                 x_(2) + u*dt];
        end
        function [Qc, Bw] = calcProcessCovarianceMatrix(obj, dt)
            Qc = diag(obj.state_sigmas.^2);
            Bw = [0 1]';
        end
        function [x, F] = propagate(obj, x_, dt, relinearize, u)
            if ~exist('u', 'var'); u = 0; end
            obj.F = obj.updateStateTransitionMatrix(x_);
            F = obj.F;
            x = obj.updateState(x_, dt, u);
        end
    end
end
