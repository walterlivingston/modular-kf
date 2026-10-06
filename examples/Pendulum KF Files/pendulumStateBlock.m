classdef pendulumStateBlock < stateblock

    properties
        g (1,1) double = -9.81
        
        J (1,1) double
        m (1,1) double
        l (1,1) double
        b (1,1) double
    end

    methods
        function [obj] = pendulumStateBlock(num_states, state_sigmas, J, m, l, b)
            obj@stateblock(num_states,state_sigmas);

            obj.J = J;
            obj.m = m;
            obj.l = l;
            obj.b = b;
        end

        function [F] = updateStateTransitionMatrix(obj, x_)
            F = [                0                          1;
                 ((obj.m*obj.g*obj.l)/obj.J)*cos(x_(1))  -(obj.b/obj.J)];
        end
        function [x] = updateState(obj, x_, dt, u)
            if ~exist('u', 'var'); u = 0; end
            x = [x_(1) + x_(2)*dt;
                 x_(2) + ((obj.m*obj.g*obj.l)*sin(x_(1))/obj.J - obj.b*x_(2)/obj.J + u/obj.J)*dt];
        end
        function [Qc, Bw] = calcProcessCovarianceMatrix(obj, dt)
            Qc = diag(obj.state_sigmas.^2);
            Bw = [0 1]';
        end
        function [x, F] = propagate(obj, x_, dt, relinearize, u)
            % u is the applied torque [Nm]
            if ~exist('relinearize', 'var'); relinearize = false; end
            if ~exist('u', 'var'); u = 0; end
            if relinearize || any(isnan(obj.F),'all')
                obj.F = obj.updateStateTransitionMatrix(x_);
                F = obj.F;
                x = obj.updateState(x_, dt, u);
            else
                F = obj.F;
                % discretize [F B] together to get the input matrix
                B = [0; 1/obj.J];
                M = expm([F B; zeros(1,3)]*dt);
                x = M(1:2,1:2)*x_ + M(1:2,3)*u;
            end
        end
    end
end

