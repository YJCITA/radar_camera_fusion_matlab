% This is a helper function and may be changed or removed without notice.

%   Copyright 2016 The MathWorks, Inc.

% Creates driving scenario display for the sensor modeling examples.
% Display is created in a new figure with 3 panels:
% # Top left corner - Top (or side) view that follows the ego vehicle
% # Bottom left corner - Chase camera view that follows the ego vehicle
% # Right side - Bird's eye plot
function [bep, figScene] = helperCreateSensorDemoDisplay(scenario,egoCar,sensor,sideView)

% By default, the top left corner is a view looking down from above (a top
% view) of the driving scenario. When sideView is true, the top left
% corner's plot is a side view of the scenario.
if nargin<4
    sideView = false;
end

% Setup figure for display. If figure doesn't exist, create one. The same
% figure is reused if it already exists.
figScene = findall(0,'Type','Figure','Tag','SensorDemoDisplay');
if isempty(figScene)
    figScene = figure('Name','Driving Scenario','Tag','SensorDemoDisplay');
    set(figScene,'Position', [0, 0, 1032, 600]); % This is the maximum figure size that will print without being clipped
    movegui(figScene, [0 -1]); % Move the figure to the left and a little down from the top
else
    figure(figScene); % If figure already exists, raise it and make it the current figure
end

% Clear the figure and recreate its panels
clf(figScene);
bep = setupPanels(scenario,egoCar,sensor,figScene,sideView);
end

% Setup the driving scenario display with three panels:
% # Top left corner - Top (or side) view that follows the ego vehicle
% # Bottom left corner - Chase camera view that follows the ego vehicle
% # Right side - Bird's eye plot
function bep = setupPanels(scenario, egoCar, sensor, figScene, sideView)

% Add a chase plot that follows the ego vehicle from behind
hCarViewPanel = uipanel(figScene, 'Position', [0 0 0.5 0.5], 'Title', 'Chase Camera View');
hCarPlot = axes(hCarViewPanel);
chasePlot(egoCar, 'Centerline', 'on', 'Parent', hCarPlot);

if sideView
    % Add a scenario plot showing the side view of the cars
    hViewPanel = uipanel(figScene, 'Position', [0 0.5 0.5 0.5], 'Title', 'Side View');
    hCarPlot = axes(hViewPanel);

    plot(scenario, 'Centerline', 'on', 'Parent', hCarPlot);
    
    % Setup side view for FCW hill descent scenario
    view(hCarPlot,[0 0]);
    hasSnapshot = snapnow('get');
    if isempty(hasSnapshot)
        xlim([0 90]);
    else
        xlim([40 90]);
    end
    pos = campos;
    pos(2) = -400;
    campos(pos);
else
    % Add a chase plot that follows the ego vehicle from a top view
    hViewPanel = uipanel(figScene, 'Position', [0 0.5 0.5 0.5], 'Title', 'Top View');
    hCarPlot = axes(hViewPanel);
    chasePlot(egoCar, 'Centerline', 'on', 'Parent', hCarPlot, 'ViewHeight', 130, 'ViewLocation', [0 0], 'ViewPitch', 80);
end

% Add a panel for a bird's-eye plot
hBEPPanel = uipanel(figScene, 'Position', [0.5 0 0.5 1], 'Title', 'Bird''s-Eye Plot');

% Create bird's-eye plot for the ego car
hBEPAxes = axes(hBEPPanel);
bep = birdsEyePlot('Parent',hBEPAxes,'XLimits',[0 50]);
axis(hBEPAxes,'equal');
axis(axis); % Freeze axis limits

xlabel(hBEPAxes,'Longitudinal Position (m)');
ylabel(hBEPAxes,'Lateral Position (m)');
zlabel(hBEPAxes,'Up Position (m)');

% Plot sensor's coverage area
sensorColor = 'blue';
caPlotter = coverageAreaPlotter(bep,'FaceColor',sensorColor,'EdgeColor',sensorColor);

plotCoverageArea(caPlotter, sensor.SensorLocation,...
    sensor.MaxRange, sensor.Yaw, sensor.FieldOfView(1));

% Create a plotter for the sensor detections. A trackPlotter is used to
% display the measurement noise covariance ellipse.
if isa(sensor,'radarDetectionGenerator')
    sensorName = 'radar';
else
    sensorName = 'vision';
end
trackPlotter(bep, 'DisplayName',sensorName,'MarkerEdgeColor',sensorColor);

% Add lane borders to plot
laneBoundaryPlotter(bep, 'DisplayName','road');

% Add an outline plotter for ground truth
outlinePlotter(bep, 'Tag', 'Ground truth');

end
