%% Driving Scenario Tutorial
% This example shows how to generate ground truth for synthetic sensor data
% and tracking algorithms.  It shows how to update actor poses in open-loop
% and closed-loop simulations.  It shows how to use the driving scenario to
% perform coordinate conversion and incorporate them into the bird's-eye
% plot.
%
%   Copyright 2016 The MathWorks, Inc.

%% Introduction
% One of the goals of a driving scenario is to generate "ground truth" test
% cases for use with sensor detection and tracking algorithms used on a
% specific vehicle.  
%
% This ground truth is typically defined in a global coordinate system;
% but, because sensors are typically mounted on a moving vehicle, this
% data needs to be converted to a reference frame that moves along with the
% vehicle.  The driving scenario facilitates this conversion automatically,
% allowing you to specify roads and paths of objects in global coordinates 
% and provides tools to convert and visualize this information in the
% reference frame of any actor in the scenario.

%% Convert Pose Information to an Actor's Reference Frame
% A |drivingScenario| consists of a model of roads and movable objects,
% called actors.  You can use actors to model pedestrians, parking meters,
% fire hydrants, and other objects within the scenario.  Actors consist of
% cuboids with a length, width, height, and a radar cross-section (RCS). An
% actor is positioned and oriented about a single point in the center of
% its bottom face.
%
% A special kind of actor that moves on wheels is a vehicle, which is
% positioned and oriented on the ground directly beneath the center of the
% rear axle, which is a more natural center of rotation.
% 
% All actors (including vehicles) may be placed anywhere within the
% scenario by specifying their respective |Position|, |Roll|, |Pitch|,
% |Yaw|, |Velocity|, and |AngularVelocity| properties.
%
% Here is an example of a scenario consisting of two vehicles 10 meters
% apart and driving towards the origin at a speed of 3 and 4 meters per
% second, respectively:

s = drivingScenario;
v1 = vehicle(s,'Position',[ 6 0 0],'Velocity',[-3  0 0],'Yaw', 180)

%%
v2 = vehicle(s,'Position',[ 0 10 0],'Velocity',[ 0 -4 0],'Yaw',-90)

%%
% To visualize a scenario, call its |plot| method:

plot(s);
set(gcf,'Name','Scenario Plot')
xlim([-20 20]);
ylim([-20 20]);

%%
% Once all the actors in a scenario have been created, you can inspect the
% pose information of all the actors in the coordinates of the scenario by
% inspecting the |Position|, |Roll|, |Pitch|, |Yaw|, |Velocity|, and
% |AngularVelocity| properties of each actor, or you may obtain all of them
% in a convenient structure by calling the |actorPoses| method of the
% scenario:

ap = actorPoses(s)

%% 
% To obtain the pose information of all other objects (or targets) seen by
% a specific actor in its own reference frame, you can call the
% |targetPoses| method on the actor itself:

v2TargetPoses = targetPoses(v2)

%%
% We can qualitatively confirm the relative vehicle placement by adding a
% chase plot for a vehicle.  By default, a chase plot displays a
% projective-perspective view from a fixed distance behind the vehicle.
% 
% Here we show the perspective seen just behind the second vehicle (red).
% The target poses seen by the second vehicle show that the location of the
% other vehicle (in blue) is 6 m forward and 10 m to the left of the second
% vehicle.  We can see this qualitatively in the chase plot:

chasePlot(v2)
set(gcf,'Name','Chase Plot')

%%
% Normally all plots associated with a driving scenario are updated in the
% course of simulation when calling the |advance| method.  If you update a
% position property of another actor manually, you can call |updatePlots|
% to see the results immediately:

v1.Yaw = 135;
updatePlots(s);

%% Convert Road Boundaries to an Actor's Reference Frame
% The driving scenario can also be used to retrieve the boundaries of roads
% defined in the scenario.  
%
% Here we make use of the simple oval track described in 
% <matlab:web(fullfile(docroot,'driving','examples','define-road-layouts.html')) Define Road Layouts>,
% which covers an area roughly 200 meters long and 100 meters wide and
% whose curves have a bank angle of nine degrees:

s = drivingScenario;
roadCenters = ...
    [  0  40  49  50 100  50  49 40 -40 -49 -50 -100  -50  -49  -40    0
     -50 -50 -50 -50   0  50  50 50  50  50  50    0  -50  -50  -50  -50
       0   0 .45 .45 .45 .45 .45  0   0 .45 .45  .45  .45  .45    0    0]';
bankAngles = ...
    [  0   0   9   9   9   9   9  0   0   9   9    9    9    9    0    0];
roadWidth = 6;     
road(s, roadCenters, roadWidth, bankAngles);
plot(s,'Centerline','on');

%%
% To obtain the lines that define the borders of the road, use the
% |roadBoundaries| method from the driving scenario.  It returns a cell
% array that contains the road borders (shown in the scenario plot above as
% the solid black lines).

rb = roadBoundaries(s)

%%
% In the example above, there are two road boundaries (an outer and an inner
% boundary).  You can plot them yourself as follows:

figure

outerBoundary = rb{1};
innerBoundary = rb{2};

plot3(innerBoundary(:,1),innerBoundary(:,2),innerBoundary(:,3),'r', ...
      outerBoundary(:,1),outerBoundary(:,2),outerBoundary(:,3),'g')
axis equal

%%
% You can use the |roadBoundaries| method of an actor to obtain the road
% boundaries in the coordinates of the actor.  To do that, simply pass the
% actor as the first argument, instead of the scenario.
%
% To see this, add an "ego car" and place it on the track:

egoCar = vehicle(s,'Position',[80 -40 .45],'Yaw',30);

%%
% Next, call the |roadBoundaries| method off of the vehicle and plot
% it as before.  It will be rendered relative to the vehicle's
% coordinates:

figure

rb = roadBoundaries(egoCar)
outerBoundary = rb{1};
innerBoundary = rb{2};

plot3(innerBoundary(:,1),innerBoundary(:,2),innerBoundary(:,3),'r', ...
      outerBoundary(:,1),outerBoundary(:,2),outerBoundary(:,3),'g')
axis equal

%% Specify Actor Paths
% You can position and plot any specific actor along a predefined 
% three-dimensional path.  
% 
% Here is an example for two vehicles that follow the racetrack at 30 m/s
% and 50 m/s respectively, each in its own respective lane.  We offset the
% cars from the center of the road by setting the offset position by half a
% lane width of 2.7 meters, and, for the banked angle sections of the
% track, half the vertical height on each side:

chasePlot(egoCar,'Centerline','on');
fastCar = vehicle(s);

d = 2.7/2;
h = .45/2;
roadOffset = [ 0  0  0  0  d  d  d  d -d -d -d -d  0  0  0  0
              -d -d -d -d  0  d  d  d  d  d  d  0 -d -d -d -d
               0  0  h  h  h  h  h  0  0  h  h  h  h  h  0  0]';

rWayPoints = roadCenters + roadOffset;
lWayPoints = roadCenters - roadOffset;

% loop around the track four times
rWayPoints = [repmat(rWayPoints(1:end-1,:),5,1); rWayPoints(1,:)];
lWayPoints = [repmat(lWayPoints(1:end-1,:),5,1); lWayPoints(1,:)];

path(egoCar,rWayPoints(:,:), 30);
path(fastCar,lWayPoints(:,:), 50);

%% Advance the Simulation
% Actors that follow a path are updated by calling |advance| on the
% driving scenario.  When |advance| is called, each actor that is following
% a path will move forward, and the corresponding plots will be updated.
% Only actors that have defined paths actually update.  This is so you can
% provide your own logic while simulation is running.
%
% The |SampleTime| property in the scenario governs the interval of time
% bewteen updates.  By default it is 10 milliseconds, but you may
% specify it with arbitrary resolution:

s.SampleTime = 0.02

%% 
% You can run the simulation by calling |advance| in the conditional of a
% while loop and placing commands to inspect or modify the scenario within
% the body of the loop.
%
% The while loop will automatically terminate when the path for any vehicle
% has finished or an optional |StopTime| has been reached.  

s.StopTime = 4;
while advance(s)
  pause(0.001)
end

%% Record a Scenario
% As a convenience when the paths of all actors are known in advance, you
% can call the |record| method of the scenario to return a structure that
% contains the pose information of each actor at each time-step.  
%
% For example, you can inspect the pose information of each actor for the
% first 100 milliseconds of the simulation, and inspect the fifth recorded
% sample:

close all

s.StopTime = 0.100;
poseRecord = record(s)

r = poseRecord(5)
r.ActorPoses(1)
r.ActorPoses(2)

%% Incorporating Multiple Views with the Bird's Eye Plot
% When debugging the simulation, you may wish to report the "ground truth"
% data in the bird's-eye plot of a specific actor while simultaneously
% viewing the plots generated by the scenario.  To do this, you can first
% create a figure with axes placed in a custom arrangement:

close all;
hFigure = figure;
hFigure.Position(3) = 900;

hPanel1 = uipanel(hFigure,'Units','Normalized','Position',[0 1/4 1/2 3/4],'Title','Scenario Plot');
hPanel2 = uipanel(hFigure,'Units','Normalized','Position',[0 0 1/2 1/4],'Title','Chase Plot');
hPanel3 = uipanel(hFigure,'Units','Normalized','Position',[1/2 0 1/2 1],'Title','Bird''s-Eye Plot');

hAxes1 = axes('Parent',hPanel1);
hAxes2 = axes('Parent',hPanel2);
hAxes3 = axes('Parent',hPanel3);

%%
% Once you have the axes defined, you specify them via the |Parent|
% property when creating the plots:

% assign a scenario plot and a chase plot in the first two axes
plot(s, 'Parent', hAxes1, 'Centerline','on');
chasePlot(egoCar, 'Parent', hAxes2,'Centerline','on');

% assign a bird's-eye plot in third axes.
egoCarBEP = birdsEyePlot('Parent',hAxes3,'XLimits',[-200 200],'YLimits',[-240 240]);
fastTrackPlotter = trackPlotter(egoCarBEP,'MarkerEdgeColor','red','DisplayName','target','VelocityScaling',.5);
egoTrackPlotter = trackPlotter(egoCarBEP,'MarkerEdgeColor','blue','DisplayName','ego','VelocityScaling',.5);
egoLanePlotter = laneBoundaryPlotter(egoCarBEP);
plotTrack(egoTrackPlotter, [0 0]);
egoOutlinePlotter = outlinePlotter(egoCarBEP);

%%
% You now can re-start the simulation and run it to completion, this time
% extracting the positional information of the target car via |targetPoses|
% and display it in the birds eye plot.  Similarly, you can also call
% |roadBoundaries| and |targetOutlines| directly from the ego-car to extract
% the road boundaries and the outlines of the actors.  The bird's-eye plot
% is capable of displaying the results of these methods directly:

restart(s)
s.StopTime = Inf;

while advance(s)
    t = targetPoses(egoCar);
    plotTrack(fastTrackPlotter, t.Position, t.Velocity);
    rbs = roadBoundaries(egoCar);
    plotLaneBoundary(egoLanePlotter, rbs);
    [position, yaw, length, width, originOffset, color] = targetOutlines(egoCar);
    plotOutline(egoOutlinePlotter, position, yaw, length, width, 'OriginOffset', originOffset, 'Color', color);
end


%% Further Information
% For more information on how to define actors and roads see:
% <matlab:web(fullfile(docroot,'driving','examples','create-actor-and-vehicle-paths.html')) Create Actor and Vehicle Paths>
% and
% <matlab:web(fullfile(docroot,'driving','examples','define-road-layouts.html')) Define Road Layouts>.
%
% For a more in-depth example on how to use the bird's-eye plot with detections and tracks see:
% <matlab:web(fullfile(docroot,'driving','examples','visualize-sensor-coverage-detections-and-tracks.html')) Visualize Sensor Coverage, Detections, and Tracks>
%
% For examples that use the driving scenario to assist in generating synthetic data see:
% <matlab:web(fullfile(docroot,'driving','examples','model-radar-sensor-detections.html'))
% Model Radar Sensor Detections>,
% <matlab:web(fullfile(docroot,'driving','examples','model-vision-sensor-detections.html'))
% Model Vision Sensor Detections>, and
% <matlab:web(fullfile(docroot,'driving','examples','sensor-fusion-using-synthetic-radar-and-vision-data.html')) Sensor Fusion Using Synthetic Radar and Vision Data>.

displayEndOfDemoMessage(mfilename)
