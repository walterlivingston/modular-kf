classdef (Abstract) stateblock < handle
    %STATEBLOCK A generic Kalman Filter State Block class
    % This class is an abstract implementation of the state block of a
    % Kalman Filter. This class is used to form the state transition and
    % process noise covariance matrices, as well as propagate the states
    % via the process model.
    %
    % state_sigmas holds the process noise parameters that
    % calcProcessCovarianceMatrix() turns into Qc, so its length matches
    % the number of columns of Bw, not the number of states.
    %
    % propagate() may take an optional control input u as a fifth
    % argument. The filter only passes u when one is given to
    % kf.process(), so blocks without an input can omit it.
    %
    % Author: Walter Livingston

    properties
        num_states      (1,1) double = 0                                % Number of States
        state_sigmas    (1,:) double = 0                                % Process Noise Parameters (sized to Bw)
        F               (:,:) double = NaN                              % State Transition Matrix
        aux             (:,:) struct = struct('X', 0, 'y', 0, 'x', 0);  % Struct of Aux Data
    end
    
    methods
        function obj = stateblock(num_states, state_sigmas)
            obj.num_states      = num_states;
            obj.state_sigmas    = state_sigmas;
        end
    end

    methods (Abstract)
        [F]         = updateStateTransitionMatrix(obj, x_);
        [x]         = updateState(obj, x_, dt);
        [Qc, Bw]    = calcProcessCovarianceMatrix(obj, dt);
        [x, F]      = propagate(obj, x_, dt, relinearize, u);
    end

    methods
        function [obj] = processAuxData(obj, aux)
            obj.aux = aux;
        end
        function [X] = applyError(obj, x_, X_, dt)
            error(['apply_error() for the state block has not ' ...
                'been implemented!']);
        end
    end
end

