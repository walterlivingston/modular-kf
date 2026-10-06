classdef (Abstract) covarianceblock < handle
    % 
    %
    % Author: Walter Livingston
    
    properties
        S (:,:) double = NaN;
        r (1,1) double = 0;
    end

    methods (Abstract)
        [S,reject]  = calcInnovationCovarianceMatrix(obj, filter, H, R);
    end
    
    methods
        function [Qc, Bw] = calcProcessCovarianceMatrix(obj, filter, dt, customStateBlock)
            if nargin >= 4 && ~isempty(customStateBlock)
                sBlock = customStateBlock;
            else
                sBlock = filter.state_block;
            end

            [Qc, Bw] = sBlock.calcProcessCovarianceMatrix(dt);
        end
        function [R] = calcMeasurementCovarianceMatrix(obj, filter, customMeasBlock)
            if nargin >= 3 && ~isempty(customMeasBlock)
                mBlock = customMeasBlock;
            else
                mBlock = filter.measurement_block;
            end
            
            R = mBlock.calcMeasurementCovarianceMatrix();
        end
    end
end

