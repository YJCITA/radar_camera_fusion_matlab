function [videoReader, videoDisplayHandle, bepPlotters, sensor] = helperCreateFCWDemoDisplay(videoFileName, sensorConfigurationFileName)
%helperCreateFCWDemoDisplay  Creates the display window for the forward collision demo
%
%   This is an example helper function and is subject to change in future
%   releases.
%
% This helper function creates the display for the forward collision
% warning example.
% Please note: a helper function may change without notice

%   Copyright 2016 The MathWorks, Inc.

% Get the video image size
videoReader = VideoReader(videoFileName);
height = videoReader.Height;
width  = videoReader.Width;
frame = readFrame(videoReader);

% Define container figure
toolBarSize = 60; % Add some pixels to allow toolbars to show on top of the figure
figurePosition = getFigurePosition(height+toolBarSize, width*2); % Twice the size to allow video and bird's-eye plot
f = figure('Position', figurePosition, 'Name', 'Forward Collision Warning With Tracking Example');

% Define the video objects
% four-element vector of form [left bottom width height]
hVideoAxes = axes(f, 'Units', 'Normal', 'Position', [0.01 0.01 0.49 0.88]);
videoDisplayHandle = createFCWDemoVideoDisplay(frame, hVideoAxes);

% Define the birdsEyePlot and plotters
bepAxes = axes(f, 'Units', 'Normal', 'Position', [0.55 0.1 0.44 0.78]);
bepPlotters = helperCreateFCWDemoBirdsEyePlot(sensorConfigurationFileName, bepAxes);

% Load the monoCamera object for on-video display. 
load('FCWDemoMonoCameraSensor.mat', 'sensor')

% Reset the video reader to the first frame
videoReader.CurrentTime = 0;
end

%% Calculate a Window Size
% The |getWindowLocation| function calculates the size of the display
% window. The window will display a video on the left hald and a bird's-eye
% plot on the right so the window width will be exactly twice the width of
% the video and will have the same height.
function figurePosition = getFigurePosition(height, width)
    screenMargin = [0, 100];   % [left, top]
    f = figure('Visible', 'off');
    defPosition = get(f, 'Position');
    figurePosition = [max(defPosition(1) + defPosition(3) - width, screenMargin(1)), max(defPosition(2) + defPosition(4) - height, screenMargin(2)), width, height];
    close(f);
end

%% Create Video Objects for the Demo 
% The |setupVideoObjects| function creates a handle to the object that
% displays the video frames on the left half of the display.
function videoFrame = createFCWDemoVideoDisplay(frame, hVideoAxes)
    % Initialize Video I/O
    % Create objects for reading a video from a file and playing the video.
    % Create a video player and display the first frame
    videoFrame = imshow(frame, [], 'Parent', hVideoAxes);
    h = title('Recorded Video');
    set(h, 'Position', [320.5, -10, 0])
end