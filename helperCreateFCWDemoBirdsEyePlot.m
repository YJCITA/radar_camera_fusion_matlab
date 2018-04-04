function bepPlotters = helperCreateFCWDemoBirdsEyePlot(sensorConfigurationFile, bepAxes)
%helperCreateFCWDemoBirdsEyePlot   Prepare Birds-Eye-Plot Display in the
%Sensor Fusion and Forward Collision Warning Demo
%
%   This is an example helper function and is subject to change in future
%   releases.
%
% Creates a birdsEyePlot object and returns a struct of birdsEyePlotter
% objects. The birdsEyePlot is shown on the right half of the display.
%
% A birdsEyePlot is a plot that is configured to use the ego-centric car
% coordinate system, where the x-axis is pointed upwards, in front of the
% car, and the y-axis is pointed to the left. 
%
% To create a birdsyEyePlot the following steps are performed:
% 
% # Read sensor positions and coverage areas from |sensorConfigurationFile|
% # Create a birdsEyePlot in the defined axes. If none are defined, they
% will be created.
% # Create coverage area plotters for the vision and radar sensors.
% # Use the coverage area plotters to plot the sensor coverages.
% # Create detection plotters for each sensor.
% # Create track plotter for all the tracks.
% # Create track plotter for the most important object (MIO).
% # Create lane boundary plotter.

%   Copyright 2016 The MathWorks, Inc.

% If no axes are specified, they will be created by bird's-eye plot
if nargin == 1
    bepAxes = [];
end

% Read the sensor configuration file
load(sensorConfigurationFile, 'sensorParams');

%Create the birds-eye plot object
BEP = birdsEyePlot('Parent',bepAxes,'XLimits',[0 120],'YLimits',[-35 35]);

%Create the sensor coverage areas (vision is first, then two radars)
cap(1) = coverageAreaPlotter(BEP,'FaceColor','blue','EdgeColor','blue');
cap(2) = coverageAreaPlotter(BEP,'FaceColor','red','EdgeColor','red');
cap(3) = coverageAreaPlotter(BEP,'FaceColor','red','EdgeColor','red');

% plot the sensor coverage areas
for i = 1:3
    plotCoverageArea(cap(i), [sensorParams(i).X, sensorParams(i).Y],...
        sensorParams(i).Range, sensorParams(i).YawAngle, sensorParams(i).FoV);
end

% create a vision detection plotter put it in a struct for future use
bepPlotters.Vision = detectionPlotter(BEP, 'DisplayName','vision detection', ...
    'MarkerEdgeColor','blue', 'Marker','^');

% we'll combine all radar detctions into one entry and store it
% for later update
bepPlotters.Radar = detectionPlotter(BEP, 'DisplayName','radar detection', ...
    'MarkerEdgeColor','red');

% Show last 10 track updates
bepPlotters.Track = trackPlotter(BEP, 'DisplayName','tracked object', ...
    'HistoryDepth',10);

% Allow for a most important object
bepPlotters.MIO = trackPlotter(BEP, 'DisplayName','most important object', ...
    'MarkerFaceColor','black');

% Lane boundaries
bepPlotters.LaneBoundary = laneBoundaryPlotter(BEP, ...
    'DisplayName','lane boundaries', 'Color',[.9 .9 0]);

% Add title
title('Birds-Eye Plot')

% Lock the legend for speedup
set(BEP.Parent.Legend, 'AutoUpdate', 'off'); 
end