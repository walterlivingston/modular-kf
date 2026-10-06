classdef vlDiscretizeTest < matlab.unittest.TestCase
    %VLDISCRETIZETEST Checks Van Loan discretization against closed forms

    methods (TestClassSetup)
        function addPaths(testCase)
            root = fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                fullfile(root, 'utils')));
        end
    end

    methods (Test)
        function doubleIntegrator(testCase)
            q = 0.3;
            T = 0.1;
            F = [0 1; 0 0];
            Bw = [0 1]';

            [Phi, Qd] = vl_discretize(F, q, Bw, T);

            testCase.verifyEqual(Phi, [1 T; 0 1], 'AbsTol', 1e-12);
            testCase.verifyEqual(Qd, q*[T^3/3 T^2/2; T^2/2 T], 'AbsTol', 1e-12);
        end

        function firstOrderMarkov(testCase)
            a = 2;
            q = 0.5;
            T = 0.25;

            [Phi, Qd] = vl_discretize(-a, q, 1, T);

            testCase.verifyEqual(Phi, exp(-a*T), 'RelTol', 1e-12);
            testCase.verifyEqual(Qd, q/(2*a)*(1 - exp(-2*a*T)), 'RelTol', 1e-12);
        end

        function qdIsSymmetricPositiveDefinite(testCase)
            F = [0 1 0; -2 -0.5 1; 0 0 -0.1];
            Bw = eye(3);
            Qc = diag([0.1 0.2 0.3]);

            [~, Qd] = vl_discretize(F, Qc, Bw, 0.05);

            testCase.verifyEqual(Qd, Qd', 'AbsTol', 1e-12);
            testCase.verifyGreaterThan(min(eig((Qd + Qd')/2)), 0);
        end
    end
end
