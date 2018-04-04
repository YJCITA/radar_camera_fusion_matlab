% This is a helper function and may be changed or removed without notice.

%   Copyright 2016 The MathWorks, Inc.

% Runs a drivingScenario simulation, update displays, and collects metrics
% from the drivng scenario using helperCollectScenarioMetrics
%
% scenario: A driving scenario object
% egoCar:   Actor in the driving scenario that is used as the ego vehicle
% sensor:   Sensor object used by egoCar to generate detections
% snapTime: Optional. Time in driving scenario when a snapshot should be
%           taken for publishing. When not publishing, this input is
%           ignored. When not specified, no snapshot is taken.
% sideView: Optional. Flag used to indicate that the upper left plot in the
%           driving scenario display should be a side view. When true, a
%           side view of the scenario is displayed, otherwise a top view
%           chasePlot following egoCar is used.
function metrics = helperRunSensorDemoScenario(scenario, egoCar, sensor, snapTime, sideView)

% Always show the display when not publishing, but when publishing, only
% show the display when it is requested
ispublishing = ~isempty(snapnow('get'));
doDisplay = nargin>3 || ~ispublishing;

if nargin<4
    snapTime = inf;
end

if nargin<5
    sideView = false;
end

% Create driving scenario display
if doDisplay
    [bep, figScene] = helperCreateSensorDemoDisplay(scenario, egoCar, sensor, sideView);
end

metrics = struct;
restart(scenario);
while advance(scenario)
    gTruth = targetPoses(egoCar); % Get target positions in ego vehicle coordinates
    
    % Generate time-stamped sensor detections
    time = scenario.SimulationTime;
    [dets, ~, isValidTime] = sensor(gTruth, time);
    
    if isValidTime
        if doDisplay
            helperUpdateSensorDemoDisplay(bep, egoCar, sensor, dets);
        end
        
        % Collect detection metrics for further analysis
        metrics = helperCollectScenarioMetrics(metrics, gTruth, dets);
    end
    
    % Take a snapshot for the published example
    if doDisplay
        helperPublishSnapshot(figScene, time>=snapTime);
    end
end
end
