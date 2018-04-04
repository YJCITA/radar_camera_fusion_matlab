function filter = helperInitSimulinkDemoFilter(detection)
%helperInitSimulinkDemoFilter Creates a tracking filter for the
%SyntheticDataSimulinkExample demo
%
% This is a helper function for example purposes and may be removed or
% modified in the future.
%
% Copyright 2017 The MathWorks, Inc.

H = [1 0 0 0 0 0; 0 0 1 0 0 0; 0 0 0 0 1 0; 0 1 0 0 0 0; 0 0 0 1 0 0; 0 0 0 0 0 1];
filter = trackingKF('MotionModel', '3D Constant Velocity', ...
    'State', H' * detection.Measurement, ...
    'MeasurementModel', H, ...
    'StateCovariance', H' * detection.MeasurementNoise * H, ...
    'MeasurementNoise', detection.MeasurementNoise);
end