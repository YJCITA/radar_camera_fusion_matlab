%% Define Road Layouts
% This example shows how to create a variety of road junctions with
% Automated Driving System Toolbox(TM).  These junctions may be combined
% with other junctions to create complicated road networks.  You can
% view the code for each plot, and use it in your own project.
%
%   Copyright 2016 The MathWorks, Inc.

%% Straight Roads
% Roads of a fixed width can be defined by a series of points that define 
% the locations of the center of the road. A straight road is very simple
% to describe by specifying its starting and stopping location. Here is an 
% example of a road which starts at (0,0) and ends at (50,0) and has a width
% of 6 (meters).

s = drivingScenario;

roadCenters = [0 0; 50 0];
roadWidth = 6;

road(s, roadCenters, roadWidth);
plot(s,'RoadCenters','on','Centerline','on');

%% Intersections
% Intersections are automatically generated wherever two roads meet.
% In this example, we add another 50 m section of road.
roadCenters = [25 -25; 25  25];
road(s, roadCenters, roadWidth);

%% Curved Roads
% Curved roads can be described by using three or more points.
% The more points you use, the more complex the curve you can create.  
% In this example, we have a curve passing through three points:

s = drivingScenario;

roadCenters = [0 0; 10 0; 53 -20];
roadWidth = 6;
         
road(s, roadCenters, roadWidth);
plot(s,'RoadCenters','on','Centerline','on');

%% Roundabouts
% When you specify the road centers, a piecewise clothoid curve is fit in
% between each segment, where curvature is preserved in between points.
% Clothoid curves are used extensively when designing roads, because they
% have a curvature that varies linearly with distance traveled along the
% road, which is very simple for drivers to navigate.
%
% By default, roads built by the scenario will have no curvature at the
% endpoints.  To make a road loop, repeat the first and last point.
%
% In this example, we show a 4m wide circular road segment circumscribed
% about a 30 m square area.  Adding roads that feed into the roundabout is
% a matter of specifying other straight or curved road segments

s = drivingScenario;

roadCenters = [-15 -15
                15 -15
                15  15
               -15  15
               -15 -15];
roadWidth = 4;         
road(s, roadCenters, roadWidth);

% Define roundabout exits with default 6 m roadwidth.
road(s, [-35   0; -20   0]);
road(s, [ 20   0;  35   0]);
road(s, [  0  35;   0  20]);
road(s, [  0 -20;  0  -35]);

plot(s,'RoadCenters','on');

%%  Exit Lane
% This example simulates a simple exit lane.  We start with a simple
% straight road and then overlay a few points of another road so that it
% overlaps the original straight road:

s = drivingScenario;

laneWidth = 3.6;

% add straight road segment
road(s, [0 0 0; 50 0 0], 2*laneWidth);

% define waypoints of lane exit
roadCenters =  [3.0 -laneWidth/2
                3.1 -laneWidth/2 
               15.0 -laneWidth
               45.0 -20];

% add the exit lane
road(s, roadCenters, laneWidth);

plot(s,'RoadCenters','on')

%% Adding Elevation
% Roads can optionally have elevation information.  This can be
% accomplished by including a third column in the waypoints.

s = drivingScenario;

roadCenters = [ 0 0 0
               25 0 3
               50 0 0];

road(s, roadCenters);

plot(s,'RoadCenters','on','Centerline','on');
view(30,24);

%% Overpasses
% Roads can cross each other without intersecting if they have 
% differing elevation.  The road surface of an overpass is typically
% 6 to 8 meters above the road.

s = drivingScenario;
roadCenters = [  0   0  0
                20 -20  0
                20  20  8
               -20 -20  8
               -20  20  0
                 0   0  0];
roadWidth = 9;
road(s, roadCenters, roadWidth);
plot(s,'RoadCenters','on','Centerline','on');
view(30,24)

%% Road Banking
% Roads can be banked; where bank angles can be defined for each waypoint.
% The following is an oval racetrack with 9 degree banked curves.
s = drivingScenario;

% transpose waypoints so they visually align with bank angles below
roadCenters = ...
    [  0  40  49  50 100  50  49 40 -40 -49 -50 -100  -50  -49  -40    0
     -50 -50 -50 -50   0  50  50 50  50  50  50    0  -50  -50  -50  -50
       0   0 .45 .45 .45 .45 .45  0   0 .45 .45  .45  .45  .45    0    0]';
bankAngles = ...
    [  0   0   9   9   9   9   9  0   0   9   9    9    9    9    0    0];
roadWidth = 6;           
road(s, roadCenters, roadWidth, bankAngles);
plot(s,'RoadCenters','on','Centerline','on');
view(-60,20)


%% Diamond Interchange
% Highways and expressways typically are comprised of two parallel roads,
% each going in the opposing direction.  An economical interchange between
% a highway and a local road is a diamond interchange, which typically
% consists of a local road overpass and four ramps.

s = drivingScenario;

% Highways
road(s, [-200 -8 0; 200 -8 0], 15); % north
road(s, [-200  8 0; 200  8 0], 15); % south

% Local Road
road(s, [-0 -200 8; 0  200 8], 10);

% Access ramps
rampNE = [ 4 -20 8;  10 -20 7.8; 126 -20 .2; 130 -20 0; 200 -13.5 0]; 
road(s, [ 1  1  1] .* rampNE, 5.4);
road(s, [ 1 -1  1] .* rampNE, 5.4);
road(s, [-1 -1  1] .* rampNE, 5.4);
road(s, [-1  1  1] .* rampNE, 5.4);

plot(s);
view(-60,30)

%% Cloverleaf Interchange
% A popular interchange between two highways is the cloverleaf interchange.
% The cloverleaf interchange consists of four inner and four outer ramps.  

s = drivingScenario;

% Highways
road(s, [-300 -8 0; 300 -8 0], 15); % north 
road(s, [-300  8 0; 300  8 0], 15); % south
road(s, [-8 -300 8; -8 300 8], 15); % east
road(s, [ 8 -300 8;  8 300 8], 15); % west

% Inner ramps
rampNE = [0 -18 0; 20 -18  0; 120 -120  4; 18  -20  8; 18 0  8]; 
rampNW = [ 1 -1 1] .* rampNE(end:-1:1,:);
rampSW = [-1 -1 1] .* rampNE;
rampSE = [ 1 -1 1] .* rampSW(end:-1:1,:);
innerRamps = [rampNE(1:end-1,:) 
              rampNW(1:end-1,:) 
              rampSW(1:end-1,:)
              rampSE];
road(s, innerRamps, 5.4);

% Outer ramps
roadCenters = [13.5 -300 8; 15 -260 8; 125 -125 4; 260 -15 0; 300 -13.5 0]; 
road(s, [ 1  1  1] .* roadCenters, 5.4);
road(s, [ 1 -1  1] .* roadCenters, 5.4);
road(s, [-1 -1  1] .* roadCenters, 5.4);
road(s, [-1  1  1] .* roadCenters, 5.4);

plot(s,'RoadCenters','on');
view(-60,30);

%% Further Information
% For an tutorial on the driving scenario see: <matlab:web(fullfile(docroot,'driving','examples','driving-scenario-tutorial.html')) Driving Scenario Tutorial>.

displayEndOfDemoMessage(mfilename)
