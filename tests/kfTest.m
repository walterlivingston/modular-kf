classdef kfTest < matlab.unittest.TestCase
    %KFTEST Checks the kf class against hand-computed Kalman filter steps

    properties
        dt = 0.1
        accel_sigma = 0.5
        meas_sigma = 0.2
    end

    methods (TestClassSetup)
        function addPaths(testCase)
            root = fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                root, 'IncludingSubfolders', true));
        end
    end

    methods
        function [f, s, m, c] = makeFilter(testCase, mode)
            s = cvStateBlock(testCase.accel_sigma);
            m = posMeasBlock(testCase.meas_sigma);
            c = basicCovBlock();
            f = kf(s, m, c, ...
                x_i = [1; 0.5], ...
                P_i = diag([4 1]), ...
                mode = mode);
        end
    end

    methods (Test)
        function timeUpdateMatchesClosedForm(testCase)
            f = testCase.makeFilter('linear');
            x0 = f.x;
            P0 = f.P;
            T = testCase.dt;
            q = testCase.accel_sigma^2;

            f.process(T);

            Phi = [1 T; 0 1];
            Qd = q*[T^3/3 T^2/2; T^2/2 T];
            testCase.verifyEqual(f.x, Phi*x0, 'AbsTol', 1e-12);
            testCase.verifyEqual(f.P, Phi*P0*Phi' + Qd, 'AbsTol', 1e-12);
        end

        function controlInputMatchesClosedForm(testCase)
            f = testCase.makeFilter('linear');
            x0 = f.x;
            T = testCase.dt;
            a = 2;

            f.process(T, u = a);

            testCase.verifyEqual(f.x, [x0(1) + x0(2)*T + a*T^2/2; x0(2) + a*T], ...
                'AbsTol', 1e-12);
        end

        function controlInputWithCustomBlock(testCase)
            % u must still be recognized after optional positional blocks
            f = testCase.makeFilter('linear');
            x0 = f.x;
            T = testCase.dt;

            f.process(T, cvStateBlock(testCase.accel_sigma), [], u = 1);

            testCase.verifyEqual(f.x(2), x0(2) + T, 'AbsTol', 1e-12);
        end

        function blocksWithoutInputStillWork(testCase)
            f = kf(noInputStateBlock(testCase.accel_sigma), ...
                posMeasBlock(testCase.meas_sigma), basicCovBlock(), ...
                x_i = [1; 0.5], P_i = diag([4 1]));

            f.process(testCase.dt);

            testCase.verifyEqual(f.x, [1 + 0.5*testCase.dt; 0.5], 'AbsTol', 1e-12);
        end

        function timeUpdateKeepsCovarianceSymmetric(testCase)
            % without symmetrizing, this drifts by ~1e-17 within 200 steps
            f = kf(coupledStateBlock(), posMeasBlock(1), basicCovBlock(), ...
                P_i = diag([4 1 0.5]) + 0.1);

            for k = 1:200
                f.process(0.05);
            end

            testCase.verifyTrue(issymmetric(f.P));
        end

        function measurementUpdateKeepsCovarianceSymmetric(testCase)
            f = kf(coupledStateBlock(), posMeasBlock(0.1), basicCovBlock(), ...
                x_i = [1; 0; 0], P_i = diag([4 1 0.5]) + 0.1);

            for k = 1:50
                f.process(0.05);
                f.update(cos(0.05*k));
                testCase.assertTrue(issymmetric(f.P), sprintf('asymmetric at step %d', k));
            end
        end

        function josephUpdateMatchesStandardForm(testCase)
            for mode = ["linear", "extended"]
                f = testCase.makeFilter(mode);
                f.process(testCase.dt);
                x0 = f.x;
                P0 = f.P;
                y = 1.3;

                f.update(y);

                H = [1 0];
                R = testCase.meas_sigma^2;
                S = H*P0*H' + R;
                L = P0*H'/S;
                testCase.verifyEqual(f.x, x0 + L*(y - H*x0), 'AbsTol', 1e-12, ...
                    "x mismatch in " + mode + " mode");
                testCase.verifyEqual(f.P, (eye(2) - L*H)*P0, 'AbsTol', 1e-12, ...
                    "P mismatch in " + mode + " mode");
                testCase.verifyEqual(f.z, y - H*x0, 'AbsTol', 1e-12);
            end
        end

        function covarianceBlockUsesPassedH(testCase)
            % posMeasBlock never stores H, so obj.H stays NaN in extended
            % mode; S must still come out right.
            [f, ~, m, c] = testCase.makeFilter('extended');
            f.process(testCase.dt);
            P0 = f.P;

            f.update(1.3);

            testCase.verifyTrue(all(isnan(m.H), 'all'));
            testCase.verifyEqual(c.S, P0(1,1) + testCase.meas_sigma^2, 'AbsTol', 1e-12);
        end

        function customBlocksAreOneShot(testCase)
            [f, s, m, c] = testCase.makeFilter('extended');
            s2 = cvStateBlock(10*testCase.accel_sigma);
            m2 = posMeasBlock(10*testCase.meas_sigma);
            c2 = basicCovBlock();

            f.process(testCase.dt, s2, c2);
            f.update(1.3, m2, c2);

            testCase.verifySameHandle(f.state_block, s);
            testCase.verifySameHandle(f.measurement_block, m);
            testCase.verifySameHandle(f.covariance_block, c);
            testCase.verifyEqual(m2.aux.y, 1.3);
        end

        function customBlockIsActuallyUsed(testCase)
            % A one-shot state block with larger noise must give a larger P.
            f1 = testCase.makeFilter('linear');
            f2 = testCase.makeFilter('linear');

            f1.process(testCase.dt);
            f2.process(testCase.dt, cvStateBlock(10*testCase.accel_sigma));

            testCase.verifyGreaterThan(f2.P(2,2), f1.P(2,2));
        end

        function emptySkipsCustomBlock(testCase)
            f1 = testCase.makeFilter('linear');
            f2 = testCase.makeFilter('linear');

            f1.process(testCase.dt);
            f2.process(testCase.dt, [], basicCovBlock());

            testCase.verifyEqual(f2.P, f1.P, 'AbsTol', 1e-12);
        end

        function rejectedMeasurementLeavesStateUnchanged(testCase)
            f = testCase.makeFilter('extended');
            f.process(testCase.dt);
            x0 = f.x;
            P0 = f.P;
            z0 = f.z;

            [~, rejected] = f.update(1.3, [], rejectAllCovBlock());

            testCase.verifyTrue(rejected);
            testCase.verifyTrue(f.rejected);
            testCase.verifyEqual(f.x, x0);
            testCase.verifyEqual(f.P, P0);
            testCase.verifyEqual(f.z, z0);

            [~, rejected] = f.update(1.3);
            testCase.verifyFalse(rejected);
            testCase.verifyFalse(f.rejected);
        end

        function rowMeasurementMatchesColumn(testCase)
            fRow = kf(cvStateBlock(testCase.accel_sigma), fullMeasBlock([0.2 0.1]), ...
                basicCovBlock(), x_i = [1; 0.5], P_i = diag([4 1]));
            fCol = kf(cvStateBlock(testCase.accel_sigma), fullMeasBlock([0.2 0.1]), ...
                basicCovBlock(), x_i = [1; 0.5], P_i = diag([4 1]));
            fRow.process(testCase.dt);
            fCol.process(testCase.dt);

            fRow.update([1.3 0.4]);
            fCol.update([1.3; 0.4]);

            testCase.verifyEqual(fRow.x, fCol.x);
            testCase.verifyEqual(fRow.P, fCol.P);
            testCase.verifyEqual(fRow.z, fCol.z);
        end

        function invalidModeErrors(testCase)
            testCase.verifyError(@() kf(cvStateBlock(1), posMeasBlock(1), ...
                basicCovBlock(), mode = "extnded"), ...
                'MATLAB:validators:mustBeMember');
        end

        function wrongSizeInitialStateErrors(testCase)
            testCase.verifyError(@() kf(cvStateBlock(1), posMeasBlock(1), ...
                basicCovBlock(), x_i = [1; 2; 3]), 'kf:stateSize');
        end

        function wrongSizeInitialCovarianceErrors(testCase)
            testCase.verifyError(@() kf(cvStateBlock(1), posMeasBlock(1), ...
                basicCovBlock(), P_i = eye(3)), 'kf:covarianceSize');
        end

        function partialNanInitialCovarianceErrors(testCase)
            testCase.verifyError(@() kf(cvStateBlock(1), posMeasBlock(1), ...
                basicCovBlock(), P_i = [1 NaN; NaN 1]), 'kf:nanCovariance');
        end

        function asymmetricInitialCovarianceErrors(testCase)
            testCase.verifyError(@() kf(cvStateBlock(1), posMeasBlock(1), ...
                basicCovBlock(), P_i = [1 0.5; 0 1]), 'kf:asymmetricCovariance');
        end

        function omittedInitialCovarianceUsesDefault(testCase)
            f = kf(cvStateBlock(testCase.accel_sigma), posMeasBlock(1), ...
                basicCovBlock());

            q = testCase.accel_sigma^2;
            testCase.verifyEqual(f.P, q*[1/3 1/2; 1/2 1], 'AbsTol', 1e-12);
        end

        function defaultCovarianceWithNoiselessStateIsFloored(testCase)
            % third state has no process noise and nothing feeding it
            s = coupledStateBlock();
            s.state_sigmas = [0.1 0.2 0];
            f = kf(s, posMeasBlock(1), basicCovBlock());

            testCase.verifyGreaterThan(min(eig(f.P)), 0);
            testCase.verifyEqual(f.P(3,3), 1e-6*max(diag(f.P)), 'RelTol', 1e-3);
        end

        function defaultCovarianceWithZeroNoiseErrors(testCase)
            s = coupledStateBlock();
            s.state_sigmas = [0 0 0];

            testCase.verifyError(@() kf(s, posMeasBlock(1), basicCovBlock()), ...
                'kf:singularDefaultCovariance');
        end

        function measurementSizeMismatchErrors(testCase)
            % H matches the scalar y, but two sigmas give a 2x2 R
            f = kf(cvStateBlock(1), posMeasBlock([1 1]), basicCovBlock(), ...
                P_i = eye(2));

            testCase.verifyError(@() f.update(1.3), 'kf:measurementSize');
        end

        function observationSizeMismatchErrors(testCase)
            % R is 2x2 and matches y, but posMeasBlock's H has only 1 row
            f = kf(cvStateBlock(1), posMeasBlock([1 1]), basicCovBlock(), ...
                P_i = eye(2));

            testCase.verifyError(@() f.update([1.3; 0.2]), 'kf:observationSize');
        end

        function nanCovarianceErrors(testCase)
            f = testCase.makeFilter('linear');
            f.P(1,2) = NaN;

            testCase.verifyError(@() f.update(1.3), 'kf:nanCovariance');
        end
    end
end
