function [visionObjects, radarObjects, inertialMeasurementUnit, laneReports, ...
    timeStep, numSteps] = helperReadSensorRecordingsFile(sensorRecordingFileName)
% Read Sensor Recordings
% The |ReadDetectionsFile| function reads the recorded sensor data file.
% The recorded data is structured in a single structure that is divided
% into the following structures:
%
% # |inertialMeasurementUnit|, a struct array with fields: timeStamp,
%   velocity, and yawRate. Each element of the array corresponds to a
%   different timestep.
% # |laneReports|, a struct array with fields: left and right. Each element
%   of the array corresponds to a different timestep.
%   Both left and right are structures with fields: isValid, confidence,
%   boundaryType, offset, headingAngle, curvature.
% # |radarObjects|, a struct array with fields: timeStamp (see below),
%   numObjects (integer) and object (struct). Each element of the array
%   corresponds to a different timestep.
%   |object| is a struct array, where each element is a separate object,
%   with the fields: id, status, position(x;y;z), velocity(vx,vy,vz),
%   amplitude, rangeMode.
%   Note: z is always constant and vz=0.
% # |visionObjects|, a struct array with fields: timeStamp (see below), 
%   numObjects (integer) and object (struct). Each element of the array 
%   corresponds to a different timestep.
%   |object| is a struct array, where each element is a separate object,
%   with the fields: id, classification, position (x;y;z),
%   velocity(vx;vy;vz), size(dx;dy;dz). Note: z=vy=vz=dx=dz=0
%
% The timeStamp for recorded vision and radar objects is a uint64 variable
% holding microseconds since the Unix epoch. Time stamps are recorded about
% 50 milliseconds apart. There is a complete synchronization between the
% recordings of vision and radar detections therefore the time stamps are
% not used in further calculations.
A = load(sensorRecordingFileName);
visionObjects = A.vision;
radarObjects = A.radar;
laneReports = A.lane;
inertialMeasurementUnit = A.inertialMeasurementUnit;

timeStep = 0.05;                 % Data is provided every 50 miliseconds
numSteps = numel(visionObjects); % Number of recorded timesteps
end