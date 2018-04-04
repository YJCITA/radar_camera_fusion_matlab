%% Visualize Sensor Coverage, Detections, and Tracks
% Configure and use a Bird's-Eye Plot to display sensor coverage,
% detections and tracking results around the ego vehicle.
%
%   Copyright 2016 The MathWorks, Inc.

%% Overview
% Displaying data recorded in vehicle coordinates on a 2-dimensional map
% around the ego car is an important part of analyzing sensor coverages,
% detections and tracking results. Use <matlab:doc('birdsEyePlot')
% birdsEyePlot> to display a snapshot of this information for a certain
% time or to stream data and efficiently update the display.
%
% This example reads pre-recorded sensor data and tracking results. It
% includes the following:
%
% * Lane information
% * Vision objects
% * Radar objects
% * Positions, velocities, covariance matrices, and labels of the tracks.
% * Most important object
%
% The above information was recorded at a high rate of 20 updates per
% second, except vision detections that were recorded at 10 updates per
% second.
%
% A sensor configuration file defines the position and coverage areas of a
% vision sensor and a radar sensor with two coverage modes. These coverage
% areas will be displayed on the bird's-eye plot.
%%
% Note that the |birdsEyePlot| object sets up a very specific 
% <matlab:helpview(fullfile(docroot,'toolbox','driving','helptargets.map'),'drivingCoordinates'); vehicle coordinate system>,
% where X-axis points forward from the vehicle and Y-axis points to the
% left of the vehicle. The origin of the coordinate system is typically
% defined as the center of the rear axle, and the positions of the sensors
% are defined relative to the origin.
%% Defining Scene Limits and Sensor Coverage
% Configuring a bird's-eye plot takes two steps. In the first step, the
% bird's-eye plot is created, which sets up the coordinate system described
% above, where the x-axis is directed upwards and y-axis is directed to the
% left. It is possible to define the axes limits in each direction. In this
% forward looking example, we define the scene up to 90 meters in front of
% the ego vehicle and 35 meters on each side.

% Create a bird's-eye plot and limit its axes
BEP = birdsEyePlot('Xlimits', [0 90], 'Ylimits', [-35 35]);

%%% 
% In the second step, the bird's-eye plotters are created. The bird's-eye
% plot offers the following variety of plotters, each configured for
% plotting a specific data type. They include:
%
% * coverageAreaPlotter - Plot sensor coverage areas
% * detectionPlotter    - Plot object detections
% * trackPlotter        - Plot tracks, track uncertainties, and history trails
% * laneBoundaryPlotter - Plot lane boundaries
% * pathPlotter         - Plot object trajectory

% Create a coverageAreaPlotter for a vision sensor and two radar modes
cap(1) = coverageAreaPlotter(BEP,'FaceColor','blue','EdgeColor','blue');
cap(2) = coverageAreaPlotter(BEP,'FaceColor','red','EdgeColor','red');
cap(3) = coverageAreaPlotter(BEP,'FaceColor','red','EdgeColor','red');

%%%
% Load sensor configuration data. Sensor configuration includes:
%
% * The position of the sensors relative to the axes origin (X,Y), in meters
% * The sensor range, in meters
% * The sensor yaw angle relative to the x-axis, in degrees
% * The sensor field of view (FOV), in degrees

load('SensorConfigurationData.mat');
% Use the sensor configuration to plot the sensor coverage areas. Vision
% sensor uses the shaded blue coverage area and radar modes are shaded in
% red.
for i = 1:3
    plotCoverageArea(cap(i), [sensorParams(i).X, sensorParams(i).Y],...
        sensorParams(i).Range, sensorParams(i).YawAngle, sensorParams(i).FoV);
end

% Add title
title('Bird''s-Eye Plot')
%%
% The display above shows the coverage of the vision sensor and two radar
% sensor modes. 
%
% The vision sensor is positioned 3.30 meters in front of the origin (rear
% axle) at the center of the car, with a range of 150 meters and a FOV of
% 38 degrees.
%
% The radar is positioned 3.38 meters in front of the origin at the center
% of the car. The radar long-range mode has a range of 174 meters and a FOV
% of 20 degrees, while the medium-range mode has a range of 60 meters and a
% FOV of 90 degrees. Note that the coverage areas are truncated at 90
% meters in front of the ego vehicle and 35 meters on each side.
%
% This example shows a forward looking scenario; however, you can define
% coverage area in $360^{\circ}$ around the ego vehicle. For example, a
% sensor that covers from the rear of the vehicle backwards would be
% oriented with a yaw angle of $180^{\circ}$.
%
% The next few lines read the recorded data in preparation for the next
% steps.

% Load recorded data from a file
load('BirdsEyePlotExampleData.mat', 'dataToDisplay');

% Skip to the 125th time step, where there are 5 vision detections and
% multiple radar objects and tracks.
timeStep = 125;

% Extract the various data from the recorded file for that time step
[visionObjectsPos, radarObjectsPos, laneBoundaries, trackPositions, ...
    trackVelocities, trackCovariances, trackLabels, MIOlabel, MIOposition, ...
    MIOvelocity] = readDataFrame(dataToDisplay(timeStep));

%% Plotting Detections
% Next, create plotters to display the recorded vision and radar detections

% create a vision detection plotter put it in a struct for future use
bepPlotters.Vision = detectionPlotter(BEP, 'DisplayName','vision detection', ...
    'MarkerEdgeColor','blue', 'Marker','^');

% we'll combine all radar detctions into one entry and store it
% for later update
bepPlotters.Radar = detectionPlotter(BEP, 'DisplayName','radar detection', ...
    'MarkerEdgeColor','red');

% Call the vision detections plotter
plotDetection(bepPlotters.Vision, visionObjectsPos);

% Repeat the above for radar detections
plotDetection(bepPlotters.Radar, radarObjectsPos);

%% Plotting Tracks and Most-Important Objects
% When adding the tracks to the Bird's-Eye Plot, we provide position,
% velocity and position covariance information. The plotter takes care of
% displaying the track history trail, but since this is a single frame,
% there will be no history.

% Create a track plotter that shows the last 10 track updates
bepPlotters.Track = trackPlotter(BEP, 'DisplayName','tracked object', ...
    'HistoryDepth',10);

% Create a track plotter to plot the most important object
bepPlotters.MIO = trackPlotter(BEP, 'DisplayName','most important object', ...
    'MarkerFaceColor','black');

% Call the track plotter to plot all the tracks
plotTrack(bepPlotters.Track, trackPositions, trackVelocities, trackCovariances, trackLabels);

% Repeat for the Most Important Object (MIO)
plotTrack(bepPlotters.MIO, MIOposition, MIOvelocity, MIOlabel);


%% Plotting the Lane Boundaries
% Plotting lane boundaries can utilize the |parabolicLaneBoundary| object.
% To use it, we saved the lane boundaries as parabolicLaneBoundary objects,
% and call the plotter with it.

% Create a plotter for lane boundaries
bepPlotters.LaneBoundary = laneBoundaryPlotter(BEP, ...
    'DisplayName','lane boundaries', 'Color',[.9 .9 0]);

% Call the lane boundaries plotter 
plotLaneBoundary(bepPlotters.LaneBoundary, laneBoundaries);

%% Displaying a Scenario from a Recording File
% The recording file contains time-dependent sensor detections, tracking
% information, and lane boundaries. The next code shows how to play back
% the recordings and display the results on the bird's-eye plot that was
% configured above.
%
% Note: vision detections were provided every other frame. In such cases,
% it is _beneficial_ to show the lack of new sensor detections. To do that,
% simply pass an empty array to the appropriate plotter to delete the
% previous detections from the display.

% Rewind to the beginning of the recording file
timeStep = 0;
numSteps = numel(dataToDisplay); % Number of steps in the scenario

% Loop through the scenario as long as the bird's eye plot is open
while timeStep < numSteps && isvalid(BEP.Parent)    
    % Promote the timeStep
    timeStep = timeStep + 1;
    
    % Capture the current time for a realistic display rate
    tic;
    
    % Read the data for that time step
    [visionObjectsPos, radarObjectsPos, laneBoundaries, trackPositions, ...
        trackVelocities, trackCovariances, trackLabels, MIOlabel, MIOposition, ...
        MIOvelocity] = readDataFrame(dataToDisplay(timeStep));

    % Plot detections
    plotDetection(bepPlotters.Vision, visionObjectsPos);    
    plotDetection(bepPlotters.Radar, radarObjectsPos);
    
    % Plot tracks and MIO
    plotTrack(bepPlotters.Track, trackPositions, trackVelocities, trackCovariances, trackLabels);
    plotTrack(bepPlotters.MIO, MIOposition, MIOvelocity, MIOlabel);
    
    % Plot lane boundaries
    plotLaneBoundary(bepPlotters.LaneBoundary, laneBoundaries);
    
    % The recorded data was obtained at a rate of 20 frames per second.
    % Pause for 50 milliseconds for a more realistic display rate. You
    % would not need this when you process data and form tracks in this
    % loop.
    pause(0.05 - toc)
end

%% Summary
% This example demonstrated how to configure and use a bird's-eye plot
% object and some of the various plotters associated with it. 
%
% Try using the track and most-important object plotters or using the
% bird's-eye plot with a different recording file.

%%% Supporting Functions
%
% *readDataFrame* - extracts the separate fields from the data provided in
% dataFrame
displayEndOfDemoMessage(mfilename)

function [visionObjectsPos, radarObjectsPos, laneBoundaries, trackPositions, ...
    trackVelocities, trackCovariances, trackLabels, MIOlabel, MIOposition, ...
    MIOvelocity] = readDataFrame(dataFrame)
    visionObjectsPos    = dataFrame.visionObjectsPos;
    radarObjectsPos     = dataFrame.radarObjectsPos;
    laneBoundaries      = dataFrame.laneBoundaries;
    trackPositions      = dataFrame.trackPositions;
    trackVelocities     = dataFrame.trackVelocities;
    trackCovariances    = dataFrame.trackCovariances;
    trackLabels         = dataFrame.trackLabels;
    MIOlabel            = dataFrame.MIOlabel;
    MIOposition         = dataFrame.MIOposition;
    MIOvelocity         = dataFrame.MIOvelocity;
end
