classdef coupledStateBlock < stateblock
    %COUPLEDSTATEBLOCK Linear 3-state block used by the unit tests
    % Its coupled dynamics make Phi*P*Phi' + Qd drift slightly asymmetric
    % in floating point, which the symmetry tests rely on.

    methods
        function [obj] = coupledStateBlock()
            obj@stateblock(3, [0.1 0.2 0.3]);
        end

        function [F] = updateStateTransitionMatrix(obj, x_)
            F = [0    1    0;
                -2   -0.5  1;
                 0    0   -0.1];
        end
        function [x] = updateState(obj, x_, dt)
            x = expm(obj.F*dt)*x_;
        end
        function [Qc, Bw] = calcProcessCovarianceMatrix(obj, dt)
            Qc = diag(obj.state_sigmas);
            Bw = eye(3);
        end
        function [x, F] = propagate(obj, x_, dt, relinearize)
            obj.F = obj.updateStateTransitionMatrix(x_);
            F = obj.F;
            x = obj.updateState(x_, dt);
        end
    end
end
