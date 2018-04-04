%% Create Actor and Vehicle Paths
% This example shows how to create actor and vehicle paths for a
% driving scenario using the Automated Driving System Toolbox(TM).  
%
%   Copyright 2016 The MathWorks, Inc.

%% Actors and Vehicles
% Actors in a driving scenario are defined as cuboid objects with a
% specific length, width, and height.  Actors also have a radar cross
% section (specified in dBsm) which can be refined by defining angular
% coordinates (azimuth and elevation).  An actor's position is defined
% as the center of the bottom face.  This point is used as the point of
% contact of the actor with the ground, as well as the actor's rotational
% center.
%
% A vehicle is a special kind of actor that moves on wheels.  Vehicles 
% possess three extra properties that govern the placement of the front
% and rear axle.  The _wheelbase_ is the distance between the front and
% rear axles.  The _front overhang_ is the amount of distance between
% the front of the vehicle and the front axle; conversely, the _rear
% overhang_ is the distance between the rear axle and the rear of the
% vehicle.  
% 
% Unlike actors, the vehicle's position is placed on the ground at the 
% center of the rear axle.  This corresponds to the vehicle's natural 
% center of rotation.  
%
% <<ActorAndVehicleCuboids.png>>
%
% A typical list of actors and their corresponding dimensions appears
% below:
%
% <<ScenarioActorChart.png>>
%
% The following example plots the position of an actor with the dimensions
% of a typical human and a vehicle in the scenario at positions (0,2) and 
% (0,-2), respectively:

s = drivingScenario;
a = actor(s, 'Length', 0.24, 'Width', 0.45, 'Height', 1.7);
passingCar = vehicle(s);
a.Position = [0  2 0]
passingCar.Position = [0 -2 0]
plot(s)
ylim([-4 4])

%%
% By default, the scenario plot will show the actors from an overhead view.
% To change this view, the scenario plot can be manipulated either by using
% the Camera Toolbar available in the "View" menu of the plot, or
% programmatically via the use of plot command line interfaces like |xlim|,
% |ylim|, |zlim|, and |view|. This allows you to qualitatively compare the
% relative heights of the actors.

zlim([0 4]);
view(-60,30)

%% Defining Paths
% You can instruct actors (including vehicles) to follow a path along a
% set of waypoints.  When you specify the waypoints, a piecewise clothoid
% curve is fit in between each segment, where curvature is preserved in
% between points.  Clothoid curves are used because they have a curvature
% that varies linearly with distance traveled, which makes a very simple 
% path for drivers to navigate when traveling at constant velocity.  
%
% By default, actor paths will have no curvature at the endpoints.  
% To complete a loop, repeat the first and last point.
%
% To follow the path at a constant velocity, specify the velocity as a
% scalar value.
%
% The vehicles rotational center pass through the curve between waypoints.
% Therefore, to accommodate the length of the vehicle in front of and
% behind the rear axle during simulation, you can offset the beginning and
% ending waypoints.  Offsetting these waypoints ensures that the vehicle
% fits completely within the road at its endpoints.
%
% If the vehicle needs to turn quickly to avoid an obstacle, place two
% points close together in the intended direction of travel.  In this
% example we show a vehicle turning quickly at two places, but otherwise
% steering normally.

s = drivingScenario;         
road(s, [0 0; 10 0; 53 -20]);
plot(s,'Waypoints','on','Centerline','on');
idleCar = vehicle(s,'Position',[25 -5.5 0],'Yaw',-22);

passingCar = vehicle(s);
waypoints = [1 -1.5; 16.36 -2.5; 17.35 -2.765; 23.83 -2.01; 24.9 -2.4; 50.5 -16.7];
velocity = 15;
path(passingCar, waypoints, velocity);

%% Turning and Braking at Intersections
% For sharp turns, define waypoints close together at the start
% and end of the turn. That way, the sudden change in steering can be
% rendered faithfully.
%
% In the example below, you can see the path that a vehicle takes when
% making a left turn.  A pair of control points are used at the beginning
% and end of the turn that define a short segment that allows for a quick
% change in the steering direction of the vehicle.
%
% To instruct vehicles to follow curves of piecewise constant acceleration,
% specify the velocities at each waypoint.  In this example below, the 
% vehicle comes decelerates from a speed of 20 m/s and comes to a brief 
% complete stop at location (-7, -1.5).  The vehicle then gradually
% accelerates back to its original speed:

s = drivingScenario;
road(s, [0 -25; 0  25],7);
road(s, [-25 0; 25 0],7);

passingCar = vehicle(s);

% take transpose so waypoints align with velocities
waypoints = [-24.0 -7.0 -3.5  -3.0  1.5  1.5  1.5
              -1.5 -1.5 -1.5  -1.5  3.0  3.5 21.0]';
velocities = [20.0  0.0  5.0   5.0 10.0 12.0 20.0];
path(passingCar, waypoints,velocities);

plot(s,'Waypoints','on','Centerline','on');

%% Moving the Vehicles
% Once all the roads, actors, and actor paths are defined, you can
% incrementally increment the position of each actor by calling the
% |advance| method of the driving scenario in a loop:
while advance(s)
    pause(0.01);
end

%% Further Information
% For an tutorial on the driving scenario see: <matlab:web(fullfile(docroot,'driving','examples','driving-scenario-tutorial.html')) Driving Scenario Tutorial>.

displayEndOfDemoMessage(mfilename)
