classdef noInputStateBlock < cvStateBlock
    %NOINPUTSTATEBLOCK State block whose propagate() takes no control input
    % Mirrors blocks written before control inputs existed, so the tests
    % catch the filter passing u to a block that doesn't accept it.

    methods
        function [obj] = noInputStateBlock(accel_sigma)
            obj@cvStateBlock(accel_sigma);
        end

        function [x, F] = propagate(obj, x_, dt, relinearize)
            obj.F = obj.updateStateTransitionMatrix(x_);
            F = obj.F;
            x = obj.updateState(x_, dt);
        end
    end
end
