%% Annotate Video Using Detections in Vehicle Coordinates
% Configure and use a |monoCamera| object to display information provided
% in vehicle coordinates on a video display.
%
%   Copyright 2016 The MathWorks, Inc.

%% Overview
% Displaying data recorded in vehicle coordinates on a recorded video is an
% integral part of ground truth labeling and analyzing tracking results.
% Using a two-dimensional bird's-eye view can help you understand the
% overall environment, but it is sometimes hard to correlate the video with
% the bird's-eye view display. In particular, this problem becomes worse
% when using a third-party sensor where you cannot access the raw video
% captured by the sensor, and you need to use a video captured by a
% separate camera.
%
% Automated Driving System Toolbox(TM) provides the <matlab:doc('monoCamera') monoCamera>
% object that facilitates the conversion between vehicle coordinates and
% image coordinates. 
% This example reads data recorded by a video sensor installed on a test
% vehicle. Then it displays the data on a video captured by a separate
% video camera installed on the same car. The data and video were recorded
% at the following rates:
%
% * Reported lane information: 20 times per second
% * Reported vision objects: 10 times per second
% * Video frame rate: 20 frames per second

%% Display a Frame with Video Annotations 
% 
% The selected frame corresponds to 5.9 seconds into the video clip, when
% there are several objects to show on the video.

% Set up video reader and player
videoFile      = '01_city_c2s_fcw_10s.mp4';
videoReader = VideoReader(videoFile);
videoPlayer = vision.DeployableVideoPlayer;

% Jump to the desired frame
time = 5.9;
videoReader.CurrentTime = time;
frameWithoutAnnotations = readFrame(videoReader);

imshow(frameWithoutAnnotations); 
title('Original Video Frame')

%%
% Get the corresponding recorded data.
recordingFile  = '01_city_c2s_fcw_10s_sensor.mat';
[visionObjects, laneReports, timeStep, numSteps] = readDetectionsFile(recordingFile);
currentStep = round(time / timeStep) + 1;
videoDetections = processDetections(visionObjects(currentStep));
laneBoundaries = processLanes(laneReports(currentStep));

% Set up the monoCamera object for on-video display
sensor = setupMonoCamera(videoReader);

frameWithAnnotations = updateDisplay(frameWithoutAnnotations, sensor, videoDetections, laneBoundaries);

imshow(frameWithAnnotations); 
title('Annotated Video Frame')

%% Display a Clip with Video Annotations
% To display the video clip with annotations, simply repeat the annotation
% frame-by-frame. The video shows that the car pitches slightly up and
% down, which changes the pitch angle. No attempt has been made to
% compensate for this pitch motion. As a result, the conversion from
% vehicle coordinates to image coordinates is a little inaccurate on some
% of the frames.

% Reset the time back to zero
currentStep = 0;                % Reset the recorded data timestep
videoReader.CurrentTime = 0;    % Reset the video reader time
while currentStep < numSteps && hasFrame(videoReader)
    % Update scenario counters
    currentStep = currentStep + 1;
    
    % Get the current time
    tic
    
    % Prepare the detections to the tracker
    videoDetections = processDetections(visionObjects(currentStep), videoDetections);

    % Process lanes
    laneBoundaries = processLanes(laneReports(currentStep));
    
    % Update video frame with annotations from the reported objects
    frameWithoutAnnotations = readFrame(videoReader);
    frameWithAnnotations = updateDisplay(frameWithoutAnnotations, sensor, videoDetections, laneBoundaries);
    
    % The recorded data was obtained at a rate of 20 frames per second.
    % Pause for 50 milliseconds for a more realistic display rate. If you
    % process data and form tracks in this loop, you do not need this
    % pause.
    pause(0.05 - toc);
    
    % Display annotated frame
    videoPlayer(frameWithAnnotations);    
end
displayEndOfDemoMessage(mfilename);
%% Create the Mono Camera for On-Video Display
% The |setupMonoCamera| function returns a |monoCamera| sensor object,
% which is used for converting positions in vehicle coordinates to image
% coordinates.
%
% Knowing the camera's intrinsic and extrinsic calibration parameters is
% critical to accurate conversion between pixel and vehicle coordinates.

%%
% Start by defining the camera intrinsic parameters. The parameters in this
% function were estimated based on the camera model. To obtain the
% parameters for your camera, use the
% <matlab:helpview(fullfile(docroot,'toolbox','vision','vision.map'),'visionCameraCalibrator'); cameraCalibrator>
% app.
%%
% Because the data in this example has little distortion, this function
% ignores the lens distortion coefficients. The parameters are next stored
% in a |cameraIntrinsics| object.
%
% Next, define the camera extrinsics. Camera extrinsics relate to the way
% the camera is mounted on the car. The mounting includes the following
% properties:
%
% * Height: Mounting height above the ground, in meters.
% * Pitch: Pitch of the camera, in degrees, where positive is angled below
%   the horizon and toward the ground. In most cases, the camera is pitched
%   slightly below the horizon.
% * Roll: Roll of the camera about its axis. For example, if the video is
%   flipped upside down, use roll = 180.
% * Yaw: Angle of the camera sideways, where positive is in the direction
%   of the positive _y_-axis (to the left). For example, a forward-facing
%   camera has a yaw angle of 0 degrees, and a backward-facing camera has
%   a yaw angle of 180 degrees.
function sensor = setupMonoCamera(vidReader)
% Define the camera intrinsics from the video information
focalLength    = [1260 1100];                    % [fx, fy]              % pixels
principalPoint = [360 245];                      % [cx, cy]              % pixels
imageSize = [vidReader.height, vidReader.width]; % [numRows, numColumns] % pixels
intrinsics = cameraIntrinsics(focalLength, principalPoint, imageSize);

% Define the camera mounting (camera extrinsics)
mountingHeight = 1.45;   % height in meters from the ground
mountingPitch  = 1.25;   % pitch of the camera in degrees
mountingRoll   = 0.15;   % roll of the camera in degrees
mountingYaw    = 0;      % yaw of the camera in degrees
sensor = monoCamera(intrinsics, mountingHeight, ...
    'Pitch', mountingPitch, ...
    'Roll',  mountingRoll, ...
    'Yaw',   mountingYaw);
end

%% Using the Mono Camera Object to Update the Display
% The |updateDisplay| function displays all the object annotations on top
% of the video frame.
%
% The display update includes the following steps:
%
% # Using the |monoCamera| sensor to convert reported detections into
%   bounding boxes and annotating the frame.
% # Using the |insertLaneBoundary| method of the |parabolicLaneBoundary|
%   object to insert the lane annotations.
function frame = updateDisplay(frame, sensor, videoDetections, laneBoundaries)

% Allocate memory for bounding boxes
bboxes = zeros(numel(videoDetections), 4);

% Create the bounding boxes
for i = 1:numel(videoDetections)
    % Use monoCamera sensor to convert the position in vehicle coordinates 
    % to the position in image coordinates. 
    % Notes: 
    %   1. The width of the object is reported and is used to calculate the
    %      size of the bounding box around the object (half width on each
    %      side). The height of the object is not reported. Instead, the 
    %      function uses a height/width ratio of 0.85 for cars and 3 for
    %      pedestrians.
    %   2. The reported location is at the center of the object at ground 
    %      level, i.e., the bottom of the bounding box.
    xyLocation1 = vehicleToImage(sensor, videoDetections(i).positions' + [0,videoDetections(i).widths/2]);
    xyLocation2 = vehicleToImage(sensor, videoDetections(i).positions' - [0,videoDetections(i).widths/2]);
    dx = xyLocation2(1) - xyLocation1(1);
    
    % Define the height/width ratio based on object class
    if strcmp(videoDetections(i).labels, 'Car')
        dy = dx * 0.85;
    elseif  strcmp(videoDetections(i).labels, 'Pedestrian')
        dy = dx * 3;
    else
        dy = dx;
    end
    
    % Estimate the bounding box around the vehicle. Subtract the height of
    % the bounding box to define the top-left corner.
    bboxes(i,:) =[(xyLocation1 - [0, dy]), dx, dy];
end

% Add labels
labels = {videoDetections(:).labels}';

% Add bounding boxes to the frame
if ~isempty(labels)
    frame = insertObjectAnnotation(frame, 'rectangle', bboxes, labels,...
        'Color', 'yellow', 'FontSize', 10, 'TextBoxOpacity', .8, 'LineWidth', 2);
end

% Display the lane boundary on the video frame
xRangeVehicle = [1, 100];
xPtsInVehicle = linspace(xRangeVehicle(1), xRangeVehicle(2), 100)';
frame = insertLaneBoundary(frame, laneBoundaries(1), sensor, xPtsInVehicle, ...
    'Color', 'red');
frame = insertLaneBoundary(frame, laneBoundaries(2), sensor, xPtsInVehicle, ...
    'Color', 'green');
end

%% Summary
% This example showed how to create a |monoCamera| sensor object and use it
% to display objects described in vehicle coordinates on a video captured
% by a separate camera. Try using recorded data and a video camera of your
% own. Try calibrating your camera to create a |monoCamera| that allows for
% transformation from vehicle to image coordinates, and vice versa.

%% Supporting Functions
%%%
% *readDetectionsFile* 
% - Reads the recorded sensor data file. The recorded data is in a single
% structure that is divided into four |struct| arrays. This example uses
% only the following two arrays:
%
% # |laneReports|, a |struct| array that reports the boundaries of the lane.
%   It has these fields: |left| and |right|.
%   Each element of the array corresponds to a different timestep.
%   Both |left| and |right| are structures with these fields: |isValid|,
%   |confidence|, |boundaryType|, |offset|, |headingAngle|, and |curvature|.
% # |visionObjects|, a struct array that reports the detected vision objects.
%   It has the fields |numObjects| (|integer|) and |object| (|struct|).
%   Each element of the array corresponds to a different timestep.
%   |object| is a |struct| array, where each element is a separate object
%   with these fields: |id|, |classification|, |position (x;y;z)|,
%   |velocity(vx;vy;vz)|, |size(dx;dy;dz)|. Note: |z=vy=vz=dx=dz=0|
function [visionObjects, laneReports, timeStep, numSteps] = readDetectionsFile(filename)
A = load(strcat(filename));
visionObjects = A.vision;
laneReports = A.lane;

% Prepare some time variables
timeStep = 0.05;                 % Lane data is provided every 50 milliseconds
numSteps = numel(visionObjects); % Number of recorded timesteps
end

%%%
% *processDetections* 
% - Reads the recorded vision detections. This example extracts only the
% following properties:
%
% # Position: A two-dimensional |[x, y]| array in vehicle coordinates
% # Width: The width of the object as reported by the video sensor (Note:
%   The sensor does not report any other dimension of the object size.)
% # Labels: The reported classification of the object
function videoDetections = processDetections(visionData, videoDetections)
% The video sensor reports a classification value as an integer
% according to the following enumeration (starting from 0)
ClassificationValues = {'Unknown', 'Unknown Small', 'Unknown Big', ...
    'Pedestrian', 'Bike', 'Car', 'Truck', 'Barrier'};

% The total number of objects reported by the sensor in this frame
numVideoObjects = visionData.numObjects;

% The video objects are reported only 10 times per second, but the video
% has a frame rate of 20 frames per second. To prevent the annotations from
% flickering on and off, this function returns the values from the previous
% timestep if there are no video objects.
if numVideoObjects == 0
    if nargin == 1 % Returning a result even if there is no previous value
        videoDetections = struct('positions', {}, 'labels', {}, 'widths', {});
    end
    return;
else
    % Prepare a container for the relevant properties of video detections
    videoDetections = struct('positions', [], 'labels', [], 'widths', []);
    for i = 1:numVideoObjects
        videoDetections(i).widths = visionData.object(i).size(2);
        videoDetections(i).positions = visionData.object(i).position(1:2);
        videoDetections(i).labels = ClassificationValues{visionData.object(i).classification + 1};
    end
end
end

%%%
% *processLanes* 
% - Reads reported lane information and converts it into
% |parabolicLaneBoundary| objects.
%
% Lane boundaries are updated based on the |laneReports| from the 
% recordings. The sensor reports the lanes as parameters of a parabolic
% model: %   $y = ax^2 + bx + c$
function laneBoundaries = processLanes(laneReports)
% Return processed lane boundaries

% Boundary type information
types = {'Unmarked', 'Solid', 'Dashed', 'Unmarked', 'BottsDots', ...
    'Unmarked', 'Unmarked', 'DoubleSolid'};

% Read the recorded lane reports for this frame
leftLane    = laneReports.left;
rightLane   = laneReports.right;

% Create parabolicLaneBoundary objects for left and right lane boundaries
leftParams = cast([leftLane.curvature, leftLane.headingAngle, leftLane.offset], 'double');
leftBoundaries = parabolicLaneBoundary(leftParams);
leftBoundaries.BoundaryType = types{leftLane.boundaryType};

rightParams  = cast([rightLane.curvature, rightLane.headingAngle, rightLane.offset], 'double');
rightBoundaries = parabolicLaneBoundary(rightParams);
rightBoundaries.BoundaryType = types{rightLane.boundaryType};

laneBoundaries = [leftBoundaries, rightBoundaries];
end
