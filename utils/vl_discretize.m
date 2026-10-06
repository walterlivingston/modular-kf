function [Phi, Qd] = vl_discretize(F,Qc,Bw,Ts)
%VL_DISCRETIZE Van Loan Discretization
%   Discretizes the state transition matrix and process noise covariance
%   matrix using Van Loan's method.
    arguments
        F   (:,:) double % Continuous State Transition Matrix
        Qc  (:,:) double % Continuous Process Covariance Matrix
        Bw  (:,:) double % Noise Input Matrix
        Ts  (1,1) double % Sampling Period
    end
    n = numel(diag(F));
    
    VLc = [       -F, Bw*Qc*Bw';
            zeros(n),        F'];
    VLd = expm(VLc*Ts);
    Phi = VLd((1+n):(2*n),(1+n):(2*n))';
    Qd = Phi * VLd(1:n,(1+n):(2*n));
end

