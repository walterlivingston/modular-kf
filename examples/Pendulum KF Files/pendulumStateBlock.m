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
            F = [                0             1;
                 ((obj.m*obj.g*obj.l)/obj.J)*x_(1)  -(obj.b/obj.J)*x_(2)];
        end
        function [x] = updateState(obj, x_, dt)
            x = [x_(1) + x_(2)*dt;
                 x_(2) + ((obj.m*obj.g*obj.l)*sin(x_(1))/obj.J - obj.b*x_(2)/obj.J)*dt];
        end
        function [Qd] = calcProcessCovarianceMatrix(obj, dt)
            Q = diag(obj.state_sigmas.^2);
            Bw = [0 1]';
            Qd = Bw*Q*Bw'.*dt;
        end
        function [x, Phi] = propagate(obj, x_, dt, relinearize)
            if ~exist('relinearize', 'var'); relinearize = false; end
            if relinearize || any(isnan(obj.F),'all')
                obj.F = obj.updateStateTransitionMatrix(x_);
                x = obj.updateState(x_, dt);
                Phi = expm(obj.F*dt);
            else
                Phi = expm(obj.F*dt);
                x = Phi*x_;
            end
        end
    end
end

