%% Model Radar Sensor Detections
% This example shows how to model and simulate the output of an automotive
% radar sensor for different driving scenarios. Generating synthetic radar
% detections is important for testing and validating tracking and sensor
% fusion algorithms in corner cases or when sensor hardware is unavailable.
% This example analyzes the differences between radar measurements and the
% vehicle ground truth position and velocity for a forward collision
% warning (FCW) scenario, a passing vehicle scenario, and a scenario with
% closely spaced targets. It also includes a comparison of signal-to-noise
% ratio (SNR) values between pedestrian and vehicle targets at various
% ranges.
%
%   Copyright 2016 The MathWorks, Inc.

%% Introduction
% Vehicles that contain advanced driver assistance system (ADAS) features
% or are designed to be fully autonomous typically rely on multiple types
% of sensors. These sensors include sonar, radar, lidar, and vision. A
% robust solution includes a sensor fusion algorithm to combine the
% strengths across the various types of sensors included in the system. For
% more information about sensor fusion of synthetic detections from a
% multisensor ADAS system, see <SensorFusionUsingSyntheticDataExample.html
% Sensor Fusion Using Synthetic Radar and Vision Data>.
%
% When using synthetic detections for testing and validating tracking and
% sensor fusion algorithms, it is important to understand how the generated
% detections model the sensor's unique performance characteristics. Each
% kind of automotive sensor provides a specific set of strengths and
% weaknesses which contribute to the fused solution. This example presents
% some important performance characteristics of automotive radars and shows
% how the radar performance is modeled by using synthetic detections.

%% Radar Sensor Model
% This example uses |radarDetectionGenerator| to generate synthetic radar
% detections. |radarDetectionGenerator| models the following performance
% characteristics of automotive radar:
%
% *Strengths*
%%
% * Good range and range-rate accuracy over long detection ranges
% * Long detection range for vehicles
%
% *Weaknesses*
%%
% * Poor position and velocity accuracy along the cross-range dimension
% * Shorter detection range for pedestrians and other nonmetallic objects
% * Close range detection clusters pose a challenge to tracking algorithms
% * Inability to resolve closely spaced targets at long ranges

%%
% _*FCW Driving Scenario*_
%%
% Create a forward collision warning (FCW) test scenario, which is used to
% illustrate how to measure a target's position with a typical long-range
% automotive radar. The scenario consists of a moving ego vehicle and a
% stationary target vehicle placed 150 meters down the road. The ego
% vehicle has an initial speed of 50 kph before applying its brakes to
% achieve a constant deceleration of 3 m/s^2. The vehicle then comes to a
% complete stop 1 meter before the target vehicle's rear bumper.

rng default;
initialDist = 150; % m
initialSpeed = 50; % kph
brakeAccel = 3;    % m/s^2
finalDist = 1;     % m
[scenario, egoCar] = helperCreateSensorDemoScenario('FCW', initialDist, initialSpeed, brakeAccel, finalDist);

%%
% _*Forward-Facing Long-Range Radar*_
%%
% Create a forward-facing long-range radar sensor mounted on the ego
% vehicle's front bumper, 20 cm above the ground. The sensor generates
% measurements every 0.1 second and has an azimuthal field of view of 20
% degrees and an angle resolution of 4 degrees. Its maximum range is 150 m
% and its range resolution is 2.5 m. The |ActorProfiles| property specifies
% the physical dimensions and radar cross-section (RCS) patterns of the
% vehicles seen by the radar in the simulation.

radarSensor = radarDetectionGenerator( ...
    'SensorIndex', 1, ...
    'UpdateInterval', 0.1, ...
    'SensorLocation', [egoCar.Wheelbase+egoCar.FrontOverhang 0], ...
    'Height', 0.2, ...
    'FieldOfView', [20 5], ...
    'MaxRange', 150, ...
    'AzimuthResolution', 4, ...
    'RangeResolution', 2.5, ...
    'ActorProfiles', actorProfiles(scenario))

%%
% _*Simulation of Radar Detections*_
%%
% Simulate the radar measuring the position of the target vehicle by
% advancing the simulation time of the scenario. The radar sensor generates
% detections from the true target pose (position, velocity, and
% orientation) expressed in the ego vehicle's coordinate frame.
%
% The radar is configured to generate detections at 0.1-second intervals,
% which is consistent with the update rate of typical automotive radars.
% However, to accurately model the motion of the vehicles, the scenario
% simulation advances every 0.01 seconds. The sensor returns a logical
% flag, |isValidTime|, that is true when the radar reaches its required
% update interval, indicating that this simulation time step will generate
% detections.

% Create display for FCW scenario
[bep, figScene] = helperCreateSensorDemoDisplay(scenario, egoCar, radarSensor);

metrics = struct;                 % Initialize struct to collect scenario metrics
while advance(scenario)           % Update vehicle positions
    gTruth = targetPoses(egoCar); % Get target positions in ego vehicle coordinates
    
    % Generate time-stamped radar detections
    time = scenario.SimulationTime;
    [dets, ~, isValidTime] = radarSensor(gTruth, time);
    
    if isValidTime
        % Update Bird's-Eye Plot with detections and road boundaries
        helperUpdateSensorDemoDisplay(bep, egoCar, radarSensor, dets);
        
        % Collect radar detections and ground truth for offline analysis
        metrics = helperCollectScenarioMetrics(metrics, gTruth, dets);
    end
    
    % Take a snapshot for the published example
    helperPublishSnapshot(figScene, time>=9.1);
end

%% Position Measurements
% Over the duration of the FCW test, the target vehicle's distance from the
% ego vehicle spans a wide range of values. By comparing the radar's
% measured longitudinal and lateral positions of the target vehicle to the
% vehicle's ground truth position, you can observe the accuracy of the
% radar's measured positions.
%
% Use |helperPlotSensorDemoDetections| to plot the longitudinal and lateral
% position errors as the difference between the measured position reported
% by the radar and the target vehicle's ground truth. The ground truth
% reference for the target vehicle is the point on the ground directly
% below the center of the target vehicle's rear axle, which is 1 meter in
% front of the car's bumper.

helperPlotSensorDemoDetections(metrics, 'position', 'reverse range', [-6 6]);

% Show rear overhang of target vehicle
tgtCar = scenario.Actors(2);
rearOverhang = tgtCar.RearOverhang;

subplot(1,2,1);
hold on; plot(-rearOverhang*[1 1], ylim, 'k'); hold off;
legend('Error', '2\sigma noise', 'Rear overhang');

%%
% _*Longitudinal Position Measurements*_
%%
% For a forward-facing radar configuration, the radar's range measurements
% correspond to the longitudinal position of the target vehicle.
%
% The longitudinal position errors in the preceding plot on the left show a
% -1 meter bias between the longitude measured by the radar and the
% target's ground truth position. This bias indicates that the radar
% consistently measures the target to be closer than the position reported
% by the ground truth. Instead of approximating the target as a single
% point in space, the radar models the physical dimensions of the vehicle's
% body. Detections are generated along the vehicle's rear side according to
% the radar's resolution in azimuth, range, and (when enabled) elevation.
% This -1 meter offset is then explained by the target vehicle's rear
% overhang, which defines the distance between the vehicle's rear side and
% its rear axle, where the ground truth reference is located.
%
% The radar is modeled with a range resolution of 2.5 meters. However, the
% $2\sigma$ measurement noise is reported to be as small as 0.25 meter at
% the closest point and grows slightly to 0.41 meter at the farthest tested
% range. The realized sensor accuracy is much smaller than the radar's
% range resolution. Because the radar models the SNR dependence of the
% range errors using the Cramer-Rao lower bound, targets with a large radar
% cross-section (RCS) or targets that are close to the sensor will have
% better range accuracy than smaller or more distant targets.
%
% This SNR dependence on the radar's measurement noise is modeled for each
% of the radar's measured dimensions: azimuth, elevation, range, and range
% rate.

%%
% _*Lateral Position Measurements*_
%%
% For a forward-facing radar configuration, the dimension orthogonal to the
% radar's range measurements (commonly referred to as the sensor's
% cross-range dimension) corresponds to the lateral position of the target
% vehicle.
%
% The lateral position errors from the FCW test in the preceding plot on
% the right show a strong dependence on the target's ground truth range.
% The radar reports lateral position accuracies as small as 0.03 meters at
% close ranges and up to 2.6 meters when the target is far from the radar.
%
% Additionally, multiple detections appear when the target is at ranges
% less than 30 meters. As previously mentioned, the target vehicle is not
% modeled as a single point in space, but the radar model compares the
% vehicle's dimensions with the radar's resolution. In this scenario, the
% radar views the rear side of the target vehicle. When the vehicle's rear
% side spans more than one of the radar's azimuth resolution cells, the
% radar generates detections from each resolution cell that the target
% occupies.
%
% Compute the azimuth spanned by the target vehicle in the FCW test when it
% is at 30 meters ground truth range from the ego vehicle.

% Range from radar to target vehicle's rear side
radarRange = 30-(radarSensor.SensorLocation(1)+tgtCar.RearOverhang);

% Azimuth spanned by vehicle's rear side at 30 meters ground truth range
width = tgtCar.Width;
azSpan = rad2deg(width/radarRange)

%%
% At a ground truth range of 30 meters, the vehicle's rear side begins to
% span an azimuth greater than the radar's azimuth resolution of 4 degrees.
% Because the azimuth spanned by the target's rear side exceeds the
% sensor's resolution, 3 resolved points along the vehicle's rear side are
% generated: one from the center of the rear side, one from the left edge
% of the rear side, and one from the right edge.

%% Velocity Measurements
% Create a driving scenario with two target vehicles (a lead car and a
% passing car) to illustrate the accuracy of a radar's longitudinal and
% lateral velocity measurements. The lead car is placed 40 meters in front
% of the ego vehicle and is traveling with the same speed. The passing car
% starts in the left lane alongside the ego vehicle, passes the ego
% vehicle, and merges into the right lane just behind the lead car. This
% merging maneuver generates longitudinal and lateral velocity components,
% enabling you to compare the sensor's accuracy along these two dimensions.
%
% Because the lead car is directly in front of the radar, it has a purely
% longitudinal velocity component. The passing car has a velocity profile
% with both longitudinal and lateral velocity components. These components
% change as the car passes the ego vehicle and moves into the right lane
% behind the lead car. Comparing the radar's measured longitudinal and
% lateral velocities of the target vehicles to their ground truth
% velocities illustrates the radar's ability to observe both of these
% velocity components.

% Create passing scenario
leadDist = 40;  % m
speed = 50;     % kph
passSpeed = 70; % kph
[scenario, egoCar] = helperCreateSensorDemoScenario('Passing', leadDist, speed, passSpeed);

%%
% _*Configuration of Radar Velocity Measurements*_
%%
% A radar generates velocity measurements by observing the Doppler
% frequency shift on the signal energy returned from each target. The rate
% at which the target's range is changing relative to the radar is derived
% directly from these Doppler frequencies. Take the radar sensor used in
% the previous section to measure position, and configure it to generate
% range-rate measurements. These measurements have a resolution of 0.5 m/s,
% which is a typical resolution for an automotive radar.

% Configure radar for range-rate measurements
release(radarSensor);
radarSensor.HasRangeRate = true;
radarSensor.RangeRateResolution = 0.5; % m/s

% Use actor profiles for the passing car scenario
radarSensor.ActorProfiles = actorProfiles(scenario);

%%
% Use |helperRunSensorDemoScenario| to simulate the motion of the ego and
% target vehicles. This function also collects the simulated metrics, as
% was previously done for the FCW driving scenario.

snapTime = 6; % Simulation time to take snapshot for publishing
metrics = helperRunSensorDemoScenario(scenario, egoCar, radarSensor, snapTime);

%%
% Use |helperPlotSensorDemoDetections| to plot the radar's longitudinal and
% lateral velocity errors as the difference between the measured velocity
% reported by the radar and the target vehicle's ground truth.

helperPlotSensorDemoDetections(metrics, 'velocity', 'time', [-25 25]);
subplot(1,2,1);
legend('Lead car error', 'Lead car 2\sigma noise', ...
    'Pass car error', 'Pass car 2\sigma noise', 'Location', 'northwest');

%%
% _*Longitudinal Velocity Measurements*_
%%
% For a forward-facing radar, longitudinal velocity is closely aligned to
% the radar's range-rate measurements. The preceding plot on the left shows
% the radar's longitudinal velocity errors for the passing vehicle
% scenario. Because the radar can accurately measure longitudinal velocity
% from the Doppler frequency shift observed in the signal energy received
% from both cars, the velocity errors for both vehicles (shown as points)
% are small. However, when the passing car enters the radar's field of view
% at 3 seconds, the passing car's $2\sigma$ measurement noise (shown using
% solid yellow lines) is initially large. The noise then decreases until
% the car merges into the right lane behind the lead car at 7 seconds. As
% the car passes the ego vehicle, the longitudinal velocity of the passing
% car includes both radial and nonradial components. The radar inflates its
% reported $2\sigma$ longitudinal velocity noise to indicate its inability
% to observe the passing car's nonradial velocity components as it passes
% the ego vehicle.

%%
% _*Lateral Velocity Measurements*_
%%
% For a forward-facing radar, the measured lateral velocity corresponds to
% a target's nonradial velocity component. The preceding plot on the right
% shows the passing car's lateral velocity measurement errors, which
% display as yellow points. The radar's inability to measure lateral
% velocity produces a large error during the passing car's lane change
% maneuver between 5 and 7 seconds. However, the radar reports a large
% $2\sigma$ lateral velocity noise (shown as solid lines) to indicate that
% it is unable to observe velocity along the lateral dimension.


%% Pedestrian and Vehicle Detection
% A radar "sees" not only an object's physical dimensions (length, width,
% and height) but also is sensitive to an object's _electrical_ size. An
% object's electrical size is referred to as its radar cross-section (RCS)
% and is commonly given in units of decibel square meters (dBsm). An
% object's RCS defines how effectively it reflects the electromagnetic
% energy received from the radar back to the sensor. An object's RCS value
% depends on many properties, including the object's size, shape, and the
% kind of materials it contains. An object's RCS also depends on the
% transmit frequency of the radar. This value can be large for vehicles and
% other metallic objects. For typical automotive radar frequencies near 77
% GHz, a car has a nominal RCS of approximately 10 square meters (10 dBsm).
% However, nonmetallic objects typically have much smaller values. -8 dBsm
% is a reasonable RCS to associate with a pedestrian. This value
% corresponds to an effective electrical size of only 0.16 square meters.
% In an ADAS or autonomous driving system, a radar must be able to generate
% detections on both of these objects.

%%
% _*FCW Driving Scenario with a Pedestrian and a Vehicle*_
%%
% Revisit the FCW scenario from earlier by adding a pedestrian standing on
% the sidewalk beside the stopped vehicle. Over the duration of the FCW
% test, the distance from the radar to the target vehicle and pedestrian
% spans a wide range of values. Comparing the radar's measured
% signal-to-noise ratio (SNR) reported for the test vehicle and pedestrian
% detections across the tested ranges demonstrates how the radar's
% detection performance changes with both detection range and object type.

% Create FCW test scenario
initialDist = 150;  % m
finalDist = 1;      % m
initialSpeed = 50;  % kph
brakeAccel = 3;     % m/s^2
withPedestrian = true;
[scenario, egoCar] = helperCreateSensorDemoScenario('FCW', initialDist, initialSpeed, brakeAccel, finalDist, withPedestrian);

%%
% _*Configuration of Radar Detection Performance*_
%%
% A radar's detection performance is usually specified by the probability
% of detecting a reference target that has an RCS of 0 dBsm at a specific
% range. Create a long-range radar that detects a target with an RCS of 0
% dBsm at a range of 100 meters, with a detection probability of 90%.

% Configure radar's long-range detection performance
release(radarSensor);
radarSensor.ReferenceRange = 100; % m
radarSensor.ReferenceRCS = 0;     % dBsm
radarSensor.DetectionProbability = 0.9;

% Use actor profiles for the passing car scenario
radarSensor.ActorProfiles = actorProfiles(scenario);

%%
% Run the scenario simulation and collect radar detections and ground truth
% data is collected for offline analysis.

snapTime = 8; % Simulation time to take snapshot for publishing
metrics = helperRunSensorDemoScenario(scenario, egoCar, radarSensor, snapTime);

%%
% Plot SNR of detections for both the target vehicle and the pedestrian.

helperPlotSensorDemoDetections(metrics, 'snr', 'range', [0 160]);
legend('Vehicle', 'Pedestrian');

%%
% This plot shows the effect of an object's RCS on the radar's ability to
% "see" it. Detections corresponding to the stationary test vehicle are
% shown in red. Detections from the pedestrian are shown in yellow.
%
% The test vehicle is detected out to the farthest range included in this
% test, but detection of the pedestrian becomes less consistent near 70
% meters. This difference between the detection range of the two objects
% occurs because the test vehicle has a much larger RCS (10 dBsm) than the
% pedestrian (-8 dBsm), which enables the radar to detect the vehicle at
% longer ranges than the pedestrian.
%
% The test vehicle is also detected at the closest range included in this
% test, but the radar stops generating detections on the pedestrian near 20
% meters. In this scenario, the target vehicle is placed directly in front
% of the radar, but the pedestrian is offset from the radar's line of
% sight. Near 20 meters, the pedestrian is no longer inside of the radar's
% field of view and cannot be detected by the radar.
%
% Revisit this scenario for a mid-range automotive radar to illustrate how
% the radar's detection performance is affected. Model a mid-range radar to
% detect an object with an RCS of 0 dBsm at a reference range of 50 meters,
% with a detection probability of 90%.

% Configure radar for a mid-range detection requirement
release(radarSensor);
radarSensor.ReferenceRange = 50; % m
radarSensor.ReferenceRCS = 0;    % dBsm
radarSensor.DetectionProbability = 0.9;

%%
% Additionally, to improve the detection of objects at close ranges that
% are offset from the radar's line of sight, the mid-range radar's
% azimuthal field of view is increased to 90 degrees. The radar's azimuth
% resolution is set to 10 degrees to search this large coverage area more
% quickly.

% Increase radar's field of view in azimuth and elevation to 90 and 10 degrees respectively
radarSensor.FieldOfView = [90 10];

% Increase radar's azimuth resolution
radarSensor.AzimuthResolution = 10;

%%
% Run the FCW test using the mid-range radar and the SNR for the detections
% from the target vehicle and pedestrian. Plot the SNR.

% Run simulation and collect detections and ground truth for offline analysis
metrics = helperRunSensorDemoScenario(scenario, egoCar, radarSensor);

% Plot SNR for vehicle and pedestrian detections
helperPlotSensorDemoDetections(metrics, 'snr', 'range', [0 160]);
legend('Vehicle', 'Pedestrian');

%%
% For the mid-range radar, the detections of both the vehicle and
% pedestrian are limited to shorter ranges. With the long-range radar, the
% vehicle is detected out to the full test range, but now vehicle detection
% becomes unreliable at 95 meters. Likewise, the pedestrian is detected
% reliably only out to 35 meters. However, the mid-range radar's extended
% field of view in azimuth enables detections on the pedestrian to a
% 10-meter ground truth range from the sensor, a significant improvement in
% coverage over the long-range radar.

%% Detection of Closely Spaced Targets
% When multiple targets occupy a radar's resolution cell, the group of
% closely spaced targets are reported as a single detection. The reported
% location is the centroid of the location of each contributing target.
% This merging of multiple targets into a single detection is common at
% long ranges, because the area covered by the radar's azimuth resolution
% grows with increasing distance from the sensor.
%
% Create a scenario with two motorcycles traveling side-by-side in front of
% the ego vehicle. This scenario shows how the radar merges closely spaced
% targets. The motorcycles are 1.8 meters apart and are traveling 10 kph
% faster than the ego vehicle.
%
% Over the duration of the scenario, the distance between the motorcycles
% and the ego vehicle increases. When the motorcycles are close to the
% radar, they occupy different radar resolution cells. By the end of the
% scenario, after the distance between the radar and the motorcycles has
% increased, both motorcycles occupy the same radar resolution cells and
% are merged. The radar's longitudinal and lateral position errors show
% when this transition occurs during the scenario.

duration = 8;         % s
speedEgo = 50;        % kph
speedMotorcyles = 60; % kph
distMotorcyles = 25;  % m
[scenario, egoCar] = helperCreateSensorDemoScenario('Side-by-Side', duration, speedEgo, speedMotorcyles, distMotorcyles);

% Create forward-facing long-range automotive radar sensor mounted on ego vehicle's front bumper
radarSensor = radarDetectionGenerator(...
    'SensorIndex', 1, ...
    'SensorLocation', [egoCar.Wheelbase+egoCar.FrontOverhang 0], ...
    'Height', 0.2, ...
    'ActorProfiles', actorProfiles(scenario));

% Run simulation and collect detections and ground truth for offline analysis
snapTime = 5.6; % Simulation time to take snapshot for publishing
metrics = helperRunSensorDemoScenario(scenario, egoCar, radarSensor, snapTime);

%%
% Plot the radar's longitudinal and lateral position errors. By analyzing
% the position errors reported for each motorcycle, you can identify the
% range where the radar no longer can distinguish the two motorcycles as
% unique objects.

helperPlotSensorDemoDetections(metrics, 'position', 'range', [-3 3], true);
subplot(1,2,2);
legend('Left error', 'Left 2\sigma noise', 'Right error', 'Right 2\sigma noise', 'Merged error', 'Merged 2\sigma noise');

%%
% Detections are generated from the rear and along the inner side of each
% motorcycle. The red errors are from the left motorcycle, the yellow
% errors are from the right motorcycle, and the purple points show the
% detections that are merged between the two motorcycles. The motorcycles
% are separated by a distance of 1.8 meters. Each motorcycle is modeled to
% have a width of 0.6 meters and a length of 2.2 meters. The inner sides of
% the motorcycles are only 1.2 meters apart.

%%
% _*Inner Side Detections*_
%%
% Detections are generated from points along the inner side of each
% motorcycle. The detections start at the closest edge and are sampled in
% range according to the radar's range resolution of 2.5 meters and the
% motorcycle's position relative to the radar. When the motorcycle occupies
% multiple range resolution cells, the closest sample points in each unique
% range cell generate detections. The location of the range cell's boundary
% produces a detection that occurs either at the middle or far edge of the
% motorcycle's inner side. A detection from the motorcycle's closest edge
% is also generated. This movement through the radar's range resolution
% cell boundaries creates the 3 bands of longitudinal position errors seen
% in the preceding plot on the left. The total longitudinal extent covered
% by these 3 bands is 2.2 meters, which corresponds to the length of the
% motorcycles.
%
% Because the inner sides of the motorcycles are separated by only 1.2
% meters, these sampled points all fall within a common azimuthal
% resolution cell and are merged by the radar. The centroid of these merged
% points lies in the middle of the two motorcycles. The centroiding of the
% merged detections produces a -0.9 m lateral bias corresponding to half of
% the distance between the motorcycles. In the lateral position error plot
% on the right, all of the merged detections (shown in purple) have this
% -0.9 m bias.

%%
% _*Rear Side Detections*_
%%
% Detections generated from the rear side of each motorcycle are further
% apart (1.8 m) than the sampled points along the inner sides (1.2 m).
%
% At the beginning of the scenario, the motorcycles are at a ground truth
% range of 25 meters from the ego vehicle. At this close range, detections
% from the rear sides lie in different azimuthal resolution cells and the
% radar does not merge them. These distinct rear-side detections are shown
% as red points (left motorcycle) and yellow points (right motorcycle) in
% the preceding longitudinal and lateral position error plots. For these
% unmerged detections, the longitudinal position errors from the rear sides
% are offset by the rear overhang of the motorcycles (0.37 m). The lateral
% position errors from the rear sides do not exhibit any bias. This result
% is consistent with the position errors observed in the FCW scenario.
%
% As the scenario proceeds, the distance between the motorcycles and the
% radar increases, and the area spanned by the radar's resolution cells
% grows. When the motorcycles move beyond a ground truth range of 39
% meters, detections generated from the rear sides of the motorcycles
% merge. Compute the azimuthal separation of the outside edges of the rear
% sides.

% Range from radar to rear side of motorcycles
motorcycle = scenario.Actors(2);
radarRange = 39-(radarSensor.SensorLocation(1)+motorcycle.RearOverhang);

% Azimuth separation between outside edges of rear sides
lateralDist = 1.8+0.6; % Total lateral distance between outer sides
azSep = rad2deg(lateralDist/radarRange)

%%
% At a ground truth range of 39 meters, the outer edges of the rear sides
% of the motorcycles now lie within the same 4 deg azimuth resolution cell
% and are merged. Beyond this range, the two motorcycles appear to the
% radar as a single object and only merged detections (shown in purple) are
% seen in the preceding plots.

%% Summary
% This example demonstrated how to model the output of automotive radars
% using synthetic detections. In particular, it presented how the
% |radarDetectionGenerator| model:
%%
% * Provides accurate longitudinal position and velocity measurements over
% long ranges, but has limited lateral accuracy at long ranges
% * Generates multiple detections from single target at close ranges, but
% merges detections from multiple closely spaced targets into a single
% detection at long ranges
% * Sees vehicles and other targets with large radar cross-sections over
% long ranges, but has limited detection performance for nonmetallic
% objects such as pedestrians

displayEndOfDemoMessage(mfilename)
