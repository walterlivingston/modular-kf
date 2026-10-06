classdef kf < handle
    %KF A generic Kalman Filter class
    % This class implements all the basic Kalman Filter equations, but uses
    % State Block and Measurement Block objects to define all the matrices
    % and state propagation functions.
    %
    % Author: Walter Livingston

    properties
        state_block         (1,1)           % State Block Object
        measurement_block   (1,1)           % Measurement Block Object
        covariance_block    (1,1)           % Covariance Block Object
        x                   (:,1) double    % State Vector
        X                   (:,1) double    % Nominal State Vector
        P                   (:,:) double    % State Covariance Matrix
        z                   (:,1) double    % Innovation Vector
        rejected            (1,1) logical = false % Last Measurement Rejected
        mode                (1,1) string    % Kalman Filter Mode
    end
    
    methods
        function obj = kf(state_block, measurement_block, covariance_block, options)
            arguments
                state_block         (1,1) stateblock
                measurement_block   (1,1) measurementblock
                covariance_block    (1,1) covarianceblock
                options.x_i         (:,1) double = zeros(state_block.num_states,1);
                options.X_i         (:,1) double = zeros(state_block.num_states,1);
                options.P_i         (:,:) double = NaN
                options.x_lin       (:,1) double = zeros(state_block.num_states,1); % Linearization Point (linear mode, must be an equilibrium)
                options.dt          (1,1) double = 1
                options.mode        (1,1) string = 'linear'
            end

            obj.state_block = state_block;
            obj.measurement_block = measurement_block;
            obj.covariance_block = covariance_block;
            obj.mode = options.mode;
            obj.x = options.x_i;

            if strcmp(obj.mode,"error")
                obj.X = options.X_i;

                sAux = obj.state_block.aux;
                sAux.X = obj.X;

                mAux = obj.measurement_block.aux;
                mAux.X = obj.X;
                
                obj.state_block.applyError(obj.x, obj.X, options.dt);
                obj.measurement_block.applyError(obj.x, obj.X);

                obj.state_block = obj.state_block.processAuxData(sAux);
                obj.measurement_block = obj.measurement_block.processAuxData(mAux);
            end
            
            if all(isnan(options.P_i))
                F = obj.state_block.updateStateTransitionMatrix(obj.x);
                [Qc, Bw] = obj.covariance_block.calcProcessCovarianceMatrix(obj, options.dt, obj.state_block);
                [~, Qd] = vl_discretize(F, Qc, Bw, options.dt);
    
                obj.P = Qd;
            else
                obj.P = options.P_i;
            end

            switch obj.mode
                case 'linear'
                    obj.state_block.F = obj.state_block.updateStateTransitionMatrix(options.x_lin);
                    obj.measurement_block.H = ...
                        obj.measurement_block.updateObservationMatrix(options.x_lin, ...
                            0);
                case 'extended'

            end
        end
        
        function [obj] = process(obj, dt, customStateBlock, customInnBlock)
        %PROCESS Implements the Time Update of a Kalman Filter
        % This function runs the time update of a Kalman filter. Using the
        % stateblock passed into the constructor, this function creates the
        % state transition and process covariance matrices and propagates
        % the state estimates and state covariance matrix. Custom blocks
        % are used for this call only and do not replace the filter's
        % blocks; pass [] to skip one.
            if nargin >= 3 && ~isempty(customStateBlock)
                sBlock = customStateBlock;
            else
                sBlock = obj.state_block;
            end

            if nargin >= 4 && ~isempty(customInnBlock)
                cBlock = customInnBlock;
            else
                cBlock = obj.covariance_block;
            end

            [obj.x, F] = sBlock.propagate(obj.x, dt, ...
                (strcmp(obj.mode, 'extended')) || (strcmp(obj.mode, 'error')));
            [Qc, Bw] = cBlock.calcProcessCovarianceMatrix(obj, dt, sBlock);
            [Phi, Qd] = vl_discretize(F, Qc, Bw, dt);
            obj.P = Phi*obj.P*Phi' + Qd;
            
            if strcmp(obj.mode,'error')
                obj.X = sBlock.applyError(obj.x, obj.X, dt);
                aux = sBlock.aux;
                aux.X = obj.X;
                sBlock.processAuxData(aux);
            end
        end

        function [obj, rejected] = update(obj, y, customMeasBlock, customInnBlock)
        %UPDATE Implements the Measurement Update of a Kalman Filter
        % Custom blocks are used for this call only and do not replace the
        % filter's blocks; pass [] to skip one. Returns (and stores in
        % obj.rejected) whether the covariance block rejected the
        % measurement.
            if nargin >= 3 && ~isempty(customMeasBlock)
                mBlock = customMeasBlock;
            else
                mBlock = obj.measurement_block;
            end

            if nargin >= 4 && ~isempty(customInnBlock)
                cBlock = customInnBlock;
            else
                cBlock = obj.covariance_block;
            end

            aux = mBlock.aux;
            aux.X = obj.X;
            aux.y = y;
            mBlock.processAuxData(aux);

            [yhat, H] = mBlock.update(obj.x, y, ...
                (strcmp(obj.mode, 'extended')) || (strcmp(obj.mode, 'error')));
            if any(isnan(obj.P), 'all')
                error('kf:nanCovariance', ...
                    'State covariance matrix P contains NaN values.');
            end
            R = cBlock.calcMeasurementCovarianceMatrix(obj, mBlock);
            oldz = obj.z;
            obj.z = (y - yhat);
            [S,reject] = cBlock.calcInnovationCovarianceMatrix(obj, H, R);
            obj.rejected = reject;
            rejected = reject;
            if ~reject
                L = obj.P*H'/S;
                I = eye(size(obj.P,1));

                obj.x = obj.x + L*obj.z;
                obj.P = (I - L*H)*obj.P*(I - L*H)' + L*R*L';

                if strcmp(obj.mode, 'error')
                    [obj.x, obj.X] = mBlock.applyError(obj.x, obj.X);
                    aux = mBlock.aux;
                    aux.X = obj.X;
                    aux.y = y;
                    mBlock.processAuxData(aux);
                end
            else
                obj.z = oldz;
            end
        end
    end
end

