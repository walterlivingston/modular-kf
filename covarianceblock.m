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
            if exist('customStateBlock', 'var')
                sBlock = customStateBlock;
            else
                sBlock = filter.state_block;
            end

            [Qc, Bw] = sBlock.calcProcessCovarianceMatrix(dt);
        end
        function [R] = calcMeasurementCovarianceMatrix(obj, filter, customMeasBlock)
            if exist('customMeasBlock', 'var')
                mBlock = customMeasBlock;
            else
                mBlock = filter.measurement_block;
            end
            
            R = mBlock.calcMeasurementCovarianceMatrix();
        end
    end
end

