classdef helperScenarioReader < driving.internal.SimulinkBusUtilities ...
        & matlab.system.mixin.CustomIcon ...
        & matlab.system.mixin.internal.SampleTime
% helperScenarioReader Read actor poses and road data from a recorded
% driving scenario
%
% This is a helper block for example purposes and may be removed or
% modified in the future.
%
% helperScenarioReader reads actor poses data saved using the record method
% of drivingScenario. The data has the following fields:
%   SimulationTime    the time of simulation 
%   ActorPoses        a struct array containing the pose of each
%                     actor at the corresponding simulation time.
%
% If the actor poses data contains the ego vehicle pose, you can specify
% which index is used for the ego data. In that case, the block will
% convert the poses of all the other actors to the ego vehicle coordinates.
% This allows the actor poses to be used by the visionDetectionGenerator
% and radarDetectionGenerator objects.
%
% If ego vehicle index is not provided, all the actor poses will be
% provided in scenario coordinates. You will then have to convert them to
% ego vehicle coordinates using the driving.scenario.targetsToEgo function
% before generating detections using the visionDetectionGenerator and
% radarDetectionGenerator objects.
%
% In addition to reading actor poses data, if you specify an ego actor, you
% can use this block to read road boundaries data in ego coordinates. Road
% boundaries in scenario coordinates are obtained using the roadBoundaries
% method of drivingScenario. They must be saved to the same file as the
% actor poses using the name RoadBoundaries.
%
% Code generation limitation for road boundaries: concatenate the road
% boundaries matrices in an N-by-3 matrix, where each road matrix (an
% Np-by-3 matrix) is separated from the next by a row of NaNs.
%
% See also: drivingScenario, drivingScenario/record,
% drivingScenario/roadBoundaries, driving.scenario.targetsToEgo,
% driving.scenario.roadBoundariesToEgo, visionDetectionGenerator,
% radarDetectionGenerator

% Copyright 2017 The MathWorks, Inc.

%#codegen
    
    % Public, tunable properties
    properties

    end

    % Public, non-tunable properties
    properties(Nontunable)
        %ScenarioFileName File name for the recorded scenario
        ScenarioFileName = char.empty(1,0);

    end
    
    properties(Constant, Hidden)
       %EgoActorSourceSet Options for ego actor source
       EgoActorSourceSet = matlab.system.StringSet({'Input port','Auto'});
    end
    
    properties(Nontunable)
        %EgoActorSource   Specify the source of the ego actor
        EgoActorSource = 'Auto'
    end
    
    properties(Nontunable, PositiveInteger)
        %EgoActorID Ego actor index in the recorded data
        EgoActorID = 1
    end
    
    properties(Nontunable, Logical)
        %RoadBoundaries Output road boundaries in ego coordinates
        RoadBoundaries = false
    end
    
    properties(Constant, Access=protected)
        %pBusPrefix Prefix used to create bus names
        %   Buses will be created with the name <pBusPrefix>#, where
        %   <pBusPrefix> is the char array set to pBusPrefix and # is an
        %   integer. Subbuses will be created by appending the name of the
        %   structure field associated with the subbus to the base bus
        %   name. For example: <pBusPrefix>#<fieldName>
        pBusPrefix = 'BusActors'
    end

    properties(DiscreteState)

    end

    % Pre-computed constants
    properties(Access = private)
        %pActors Contains the loaded actors data
        pActors
        
        %pRoads  Contains the loaded road boundaries data in scenario frame
        pRoads

        %pCurrentTime Current simulation time
        pCurrentTime
        
        %pCurrentIndex Current index into the actors data struct
        pCurrentIndex
        
        %pMaxIndex     Saved the maximum time step index in the data file
        pMaxIndex
        
        %pRoadBoundariesSize Specifies the size of the road boundaries output for codegen
        pRoadBoundariesSize
    end
    properties(Access = private, Nontunable)
        %pHasEgoActor A logical value. True if the ego actor is in file
        pHasEgoActor
    end

    methods
        % Constructor
        function obj = helperScenarioReader(varargin)
            % Support name-value pair arguments when constructing object
            setProperties(obj,nargin,varargin{:})
        end
    end

    methods(Access = protected)
        %% Common functions
        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants
            obj.pHasEgoActor = strcmpi(obj.EgoActorSource, 'Auto');
            
            % Reread the road boundaries from the file
            if obj.RoadBoundaries
                readRoadBoundriesFromFile(obj);
            end
        end
        
        function flag = checkStructForFields(~, s, expFields)
            % Returns true if the struct s has the expected fields, expFields
            
            flag = true;
            for i = 1:numel(expFields)
                if ~isfield(s,expFields{i})
                    flag = false;
                    return
                end
            end
        end

        function [posesOnBus, varargout] = stepImpl(obj, varargin)
            % Implement algorithm. Calculate y as a function of input u and
            % discrete states.
            
            % Find the data index based on the simulation time
            obj.pCurrentTime = obj.pCurrentTime + getSampleTime(obj);
            while abs(obj.pActors(obj.pCurrentIndex).SimulationTime - obj.pCurrentTime) > 1e-4
                if ~obj.isEndOfData()
                    obj.pCurrentIndex = obj.pCurrentIndex + 1;
                else
                    break
                end
            end
            
            % Read the actor poses
            if obj.pHasEgoActor
                egoCar = obj.pActors(obj.pCurrentIndex).ActorPoses(obj.EgoActorID);
                otherActorIndices = (obj.EgoActorID ~= 1:numel(obj.pActors(obj.pCurrentIndex).ActorPoses));
                actors = obj.pActors(obj.pCurrentIndex).ActorPoses(otherActorIndices);
            else
                egoCar = varargin{1};
                actors = obj.pActors(obj.pCurrentIndex).ActorPoses;
            end
            actorPoses = driving.scenario.targetsToEgo(actors, egoCar);
            posesOnBus = sendToBus(obj,actorPoses,obj.pCurrentIndex);
            
            if obj.RoadBoundaries
                roads = driving.scenario.roadBoundariesToEgo(obj.pRoads, egoCar);
                varargout = {roads(1:obj.pRoadBoundariesSize(1),1:obj.pRoadBoundariesSize(2))};
            end
        end

        function resetImpl(obj)
            % Initialize / reset discrete-state properties
            
            % Reread the actors data from the file
            readActorsDataFromFile(obj);
            
            % Initialize the time
            if coder.target('MATLAB')
                obj.pCurrentTime = str2double(get_param(bdroot,'StartTime'));
            else
                obj.pCurrentTime = 0;
            end
            obj.pCurrentIndex = 1;
        end

        function validatePropertiesImpl(~)
%             % Validate related or interdependent property values
        end

        function flag = isInactivePropertyImpl(obj,prop)
            % Return false if property is visible based on object 
            % configuration, for the command line and System block dialog
            
            flag = isInactivePropertyImpl@driving.internal.SimulinkBusUtilities(obj,prop);
            hasActor = strcmpi(obj.EgoActorSource,'Input port');
            
            if strcmp(prop,'EgoActorID') && hasActor
                flag = true;
            end
        end

        %% Backup/restore functions
        function s = saveObjectImpl(obj)
            % Set properties in structure s to values in object obj

            % Set public properties and states
            s = saveObjectImpl@driving.internal.SimulinkBusUtilities(obj);

            % Set private and protected properties
            s.pActors               = obj.pActors;
            s.pRoads                = obj.pRoads;
            s.pCurrentTime          = obj.pCurrentTime;
            s.pCurrentIndex         = obj.pCurrentIndex;
            s.pHasEgoActor          = obj.pHasEgoActor;
            s.pMaxIndex             = obj.pMaxIndex;
            s.pRoadBoundariesSize   = obj.pRoadBoundariesSize;
        end

        function status = isEndOfData(obj)
            % Return true if end of data has been reached
            status = (obj.pCurrentIndex == obj.pMaxIndex);
        end

        function loadObjectImpl(obj,s,wasLocked)
            % Set properties in object obj to values in structure s

            % Set private and protected properties
            obj.pActors             = s.pActors;
            obj.pRoads              = s.pRoads;
            obj.pCurrentTime        = s.pCurrentTime;
            obj.pCurrentIndex       = s.pCurrentIndex;
            obj.pHasEgoActor        = s.pHasEgoActor;
            obj.pMaxIndex           = s.pMaxIndex;
            obj.pRoadBoundariesSize = s.pRoadBoundariesSize;

            % Set public properties and states
            loadObjectImpl@driving.internal.SimulinkBusUtilities(obj,s,wasLocked);
        end

        %% Simulink functions
        function ds = getDiscreteStateImpl(~)
            % Return structure of properties with DiscreteState attribute
            ds = struct([]);
        end

        function validateInputsImpl(obj,varargin)
            % Validate inputs to the step method at initialization
            if strcmpi(obj.EgoActorSource, 'Input port') % Ego actor comes from input
                egoActor = varargin{1};
                validateattributes(egoActor,{'struct'},{'scalar'},'helperScenarioReader','Ego actor input');
                expectedFields = {'ActorID','Position','Velocity','Roll','Pitch','Yaw','AngularVelocity'};
                expectedNumels = [1,3,3,1,1,1,3];
                for i=1:numel(expectedFields)
                    coder.internal.errorIf(~isfield(egoActor,expectedFields{i}),...
                        ['Expected ego actor to have field ', expectedFields{i}]);
                    validateattributes(egoActor.(expectedFields{i}), {'numeric'},...
                        {'numel',expectedNumels(i)},'helperScenarioReader', ['Ego actor',expectedFields{i}]);
                end
            end
        end

        function flag = isInputSizeLockedImpl(~,~)
            % Return true if input size is not allowed to change while
            % system is running
            flag = true;
        end

        function num = getNumInputsImpl(obj)
            % Define total number of inputs for system with optional inputs
            num = 0;
            if strcmpi(obj.EgoActorSource, 'Input port')
                num = 1;
            end
        end

        function num = getNumOutputsImpl(obj)
            % Define total number of outputs for system with optional
            % outputs
            num = 1;
            if obj.RoadBoundaries
                num = 2;
            end
        end

        function [out,varargout] = getOutputSizeImpl(obj)
            % Return size for each output port
            out = [1 1];

            if obj.RoadBoundaries
                readRoadBoundriesFromFile(obj);
                ego = struct('Position',[0 0 0],'Yaw',0,'Pitch',0,'Roll',0);
                rbEgo = driving.scenario.roadBoundariesToEgo(obj.pRoads, ego);
                rbSize = size(rbEgo);
                varargout = {rbSize}; 
            end
        end

        function [out, varargout] = getOutputDataTypeImpl(obj)
            % Return data type for each output port
            out = getOutputDataTypeImpl@driving.internal.SimulinkBusUtilities(obj);
            
            % Set the simulation to stop at the latest time stamp or
            % earlier while keeping the Dirty flag state
            lastTime = obj.pActors(end).SimulationTime;
            stopTime = str2double(get_param(bdroot,'StopTime'));
            epsilon = 1e-5;
            if lastTime<(stopTime-epsilon)
                Simulink.output.error(['Simulation stop time must not be ',...
                    'greater than the last time stamp of actor poses data ',...
                    'in the recorded file, which is ', num2str(lastTime)]);
            end
            
            if obj.RoadBoundaries
                varargout = {'double'};
            end
        end

        function [out, varargout] = isOutputComplexImpl(obj)
            % Return true for each output port with complex data
            out = false;
            if obj.RoadBoundaries
                varargout = {false};
            end
        end

        function [out, varargout] = isOutputFixedSizeImpl(obj)
            % Return true for each output port with fixed size
            out = true;
            if obj.RoadBoundaries
                varargout = {false};
            end
        end
        
        function readActorsDataFromFile(obj)
            % Try to load the file name. Error out if the file does not
            % exist or if it does not contain the expected format of
            % recorded actors data
            s = coder.load(obj.ScenarioFileName);
            
            % Search for the field that has the recorded data format
            validateattributes(s, {'struct'}, {'nonempty'},'ScenarioReader');
            expFields = {'SimulationTime','ActorPoses'};
            flag = checkStructForFields(obj, s, expFields);
            foundActors = false;
            if flag
                foundActors = true;
                obj.pActors = s;
            else % Maybe the actors are a field inside s
                fields = fieldnames(s);
                for i = 1:numel(fields)
                    flag = checkStructForFields(obj, s.(fields{i}), expFields);
                    if flag
                        foundActors = true;
                        obj.pActors = s.(fields{i});
                        break
                    end
                end
            end
            obj.pMaxIndex = numel(obj.pActors);
            if ~foundActors % helper object: no need for a message catalog
                error(['Couldn''t find actor data in the file ', obj.ScenarioFileName])
            end
        end
        
        function readRoadBoundriesFromFile(obj)
            % Try to load the file name. Error out if the file does not
            % exist or if it does not contain the expected format of
            % recorded actors data
            s = coder.load(obj.ScenarioFileName);
            
            % Search for the field that has the recorded data format
            validateattributes(s, {'struct'}, {'nonempty'},'ScenarioReader');
            expFields = {'RoadBoundaries'};
            flag = checkStructForFields(obj, s, expFields);
            foundRoads = false;
            if flag
                foundRoads = true;
                roads = s.RoadBoundaries;
            else % Maybe the actors are a field inside s
                fields = fieldnames(s);
                for i = 1:numel(fields)
                    flag = checkStructForFields(obj, s.(fields{i}), expFields);
                    if flag
                        foundRoads = true;
                        roads = s.(fields{i});
                        break
                    end
                end
            end
            if ~foundRoads % helper object: no need for a message catalog
                error(['Couldn''t find road boundaries data in the file ', obj.ScenarioFileName])
            else
                if isstruct(roads)
                    r = struct2cell(roads);
                    numRoads = numel(r);
                    obj.pRoads = reshape(r,1,numRoads);
                elseif iscell(roads)
                    obj.pRoads = roads;
                else
                    obj.pRoads = {roads};
                end
            end
            egoCar = struct('ActorID',1, 'Position', [0 0 0], ...
                'Velocity', [0 0 0], 'Roll', 0, 'Pitch', 0, 'Yaw', 0, ...
                'AngularVelocity', [0 0 0]);
            roads = driving.scenario.roadBoundariesToEgo(obj.pRoads, egoCar);
            obj.pRoadBoundariesSize = size(roads);
        end
        
        %---------------------------------
        % Simulink bus propagation methods
        %---------------------------------
        function [out, argsToBus] = defaultOutput(obj)
        % Default output that will be placed on the bus. argsToBus are any
        % additional inputs that will be passed when sendToBus is called.
            
            readActorsDataFromFile(obj);
            numActors = numel(obj.pActors(1).ActorPoses) - strcmpi(obj.EgoActorSource, 'Auto');
            actorStruct = obj.pActors(1).ActorPoses(1);
            actorPoses = repmat(actorStruct, [numActors,1]);
            out = actorPoses;
            argsToBus = {1};
        end
        
        function outStruct = sendToBus(obj, actorPoses, varargin)
        % Converts the output, out, from the defaultOutput method above to
        % a struct or array of structs which will be sent out on the
        % Simulink bus. A typical output struct has the following
        % structure:
        %   Num<Name>:  The number of valid elements in the output, where
        %               Name describes the type of output, such as
        %               "Detections" or "Tracks"
        %   <Name>:     A struct array formed from the output where
        %               output can be a cell array of objects to be
        %               converted to an array of structs to be placed on
        %               the bus.
            index = varargin{1};
            outStruct = struct('NumActors', numel(actorPoses), ...
                'Time', obj.pActors(index).SimulationTime, ...
                'Actors', actorPoses);
        end
        
        function icon = getIconImpl(~)
            % Define icon for System block
            icon = {'Scenario';'Reader'};
        end

        function varargout = getInputNamesImpl(obj)
            % Return input port names for System block
            varargout = cell(1,nargout);
            if strcmpi(obj.EgoActorSource,'Input port')
                varargout{1} = sprintf('Ego Actor');
            end
        end

        function [name, varargout] = getOutputNamesImpl(obj)
            % Return output port names for System block
            name = 'Actors';
            if obj.RoadBoundaries
                varargout = {sprintf('Roads')};
            end
        end
    end

    methods(Static, Access = protected)
        %% Simulink customization functions
        function header = getHeaderImpl
            % Define header panel for System block dialog
            header = matlab.system.display.Header(...
                'Title', 'ScenarioReader', ...
                'Text', getHeaderText());
        end

        function groups = getPropertyGroupsImpl
            % Define property section(s) for System block dialog
            group = matlab.system.display.Section(...
                'Title','Scenario', ...
                'PropertyList', {'ScenarioFileName', 'EgoActorSource', ...
                    'EgoActorID', 'RoadBoundaries'});
            busUtil = getPropertyGroupsImpl@driving.internal.SimulinkBusUtilities;            
            groups = [group,busUtil];
        end
    end
end

function str = getHeaderText
str = sprintf([...
    'The Scenario Reader reads actor poses data saved using the ',...
    'drivingScenario record method and road boundary data saved as structs ',...
    'from the drivingScenario roadBoundaries method.\n\n', ...
    'For open loop simulation, the ego actor pose must be saved to the ',...
    'scenario file. Choose the ''Auto'' for the source of ego actor pose ',...
    'to specify the ego actor index in the recorded data.\n',...
    'For closed loop simulation, the ego actor pose can be provided from ',...
    'an input port. Choose the ''Input port'' for the source of the ego ',...
    'actor pose and pass a valid actor pose to it.']); 
end