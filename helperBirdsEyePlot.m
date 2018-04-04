classdef helperBirdsEyePlot < matlab.System & matlab.system.mixin.CustomIcon
    % helperBirdsEyePlot Creates and updates a bird's-eye plot display 
    %
    % This is a helper block for example purposes and may be removed or
    % modified in the future.
    %
    % helperBirdsEyePlot creates and maintains a bird's-eye plot. You can
    % specify if the bird's-eye plot displays actors, vision detections,
    % radar detections, and tracks. 
    % When the vision and radar detections input ports are enabled, the
    % helperBirdsEyePlot block will automatically create the coverage
    % areas and plotters for all the visionDetectionGenerator and
    % radarDetectionGenerator in the model.
    %
    % The bird's-eye plot figure can only be closed when the simulation is
    % stopped and 'FastRestart' is disabled.
    %
    % The bird's-eye plot block is for display purposes and you will have
    % to disable it to set the model in 'Rapid Accelerator' mode.
    %
    % See also: birdsEyePlot, visionDetectionGenerator,
    % radarDetectionGenerator, multiObjectTracker
    
    % Copyright 2017 The MathWorks, Inc.
    
    %#codegen
    
    properties(Nontunable)
        %XLim Minimum and maximum distance in the longitudinal axis
        XLim = [0 60]
        
        %YLim Minimum and maximum distance in the lateral axis
        YLim = [-30 30]
    end
    
    properties(Nontunable, Logical)
        %HasActors Display actors data
        HasActors = true
    
        %HasVision Display vision detection generator data
        HasVision = false
    
        %HasRadar Display radar detection generator data
        HasRadar = false
    
        %HasTracks Display tracks data
        HasTracks = false
    end
    
    properties (Nontunable)
        %TrackPositionSelector Select x and y from the state vector
        %  For example, if the state is [x;vx;y;vy;z;vz], [x;y] = H * state
        %  Where H = [1 0 0 0 0 0; 0 0 1 0 0 0]
        TrackPositionSelector = [1 0 0 0 0 0; 0 0 1 0 0 0]; 
        
        %TrackVelocitySelector Select vx and vy from the state vector
        %  For example, if the state is [x;vx;y;vy;z;vz], [vx;vy] = H * state
        %  Where H = [0 1 0 0 0 0; 0 0 0 1 0 0]
        TrackVelocitySelector = [0 1 0 0 0 0; 0 0 0 1 0 0]; 
    end
       
    properties(Nontunable, Logical)
        %HasMIO   Display the most important object
        HasMIO = false
        
        %HasRoads Display road boundary data
        HasRoads = false
    end
    
    properties(Access=private)
        pFig
        pBEP
        pActorPlotter
        pVisionPlotter
        pRadarPlotter
        pTrackPlotter
        pMIOPlotter
        pActorProfile
        pLaneBoundaryPlotter
        pSimulinkUIToolbar
        pBlockName
        pIsLegendOn
    end
    
    methods
        function obj = helperBirdsEyePlot(varargin)
            % Constructor
            setProperties(obj,nargin,varargin{:});
        end
    end
    
    methods(Access=protected)
        function setupImpl(obj)
            wasFigureClosed = (isempty(obj.pFig) || ~ishghandle(obj.pFig));
            obj.pBlockName = gcb;
            if wasFigureClosed
                % Find the hidden figure handle
                root = groot;
                shh = get(root,'ShowHiddenHandles');
                set(root,'ShowHiddenHandles','on');
                hfig = findobj('Tag',obj.pBlockName);
                set(root,'ShowHiddenHandles',shh);
                if isempty(hfig)
                    hfig = figure('Name',obj.pBlockName,'Tag',obj.pBlockName, ...
                        'CloseRequestFcn',@helperCloseReq,'Visible','off');
                    obj.pIsLegendOn = true;
                else % Hide the figure while we build the bird's-eye plot
                    set(hfig,'Visible','off')
                    obj.pIsLegendOn = ~isempty(get(hfig.CurrentAxes,'Legend'));
                end

                obj.pFig = hfig;
            end
            
            % Create BEP before the toolbar because clf clears toolbars
            isBEPNeeded = (isempty(obj.pBEP) || wasFigureClosed);
            if isBEPNeeded
                clf(obj.pFig);
                hax = axes(obj.pFig);
                obj.pBEP = birdsEyePlot('Parent',hax,'XLim',obj.XLim,'YLim',obj.YLim);
                
                if obj.HasActors
                    obj.pActorPlotter = outlinePlotter(obj.pBEP);
                    obj.pActorProfile = struct( ...
                        'Length', 4.7, ...
                        'Width', 1.8, ...
                        'OriginOffset', [-1.35 0]);
                end
                
                if obj.HasVision
                    obj.pVisionPlotter = detectionPlotter(obj.pBEP,...
                        'DisplayName','Vision Detections',...
                        'MarkerEdgeColor', 'blue',...
                        'MarkerFaceColor', 'blue');
                end
                
                if obj.HasRadar
                    obj.pRadarPlotter = detectionPlotter(obj.pBEP,...
                        'DisplayName','Radar Detections', ...
                        'MarkerEdgeColor', 'red', ...
                        'MarkerFaceColor', 'red');
                end
                
                if obj.HasTracks
                    obj.pTrackPlotter = trackPlotter(obj.pBEP,'DisplayName','Tracks','HistoryDepth',7);
                    if obj.HasMIO % MIO can only appear if tracks show
                        obj.pMIOPlotter = trackPlotter(obj.pBEP,'DisplayName','Most Important Object','HistoryDepth',0,'MarkerSize',10,'MarkerFaceColor','yellow');
                    end
                end
                
                if obj.HasRoads
                    grey = 0.3*ones(1,3);
                    obj.pLaneBoundaryPlotter = laneBoundaryPlotter(obj.pBEP,'DisplayName','Roads', 'Color', grey);
                end
            end
        end
        
        function resetImpl(obj)
            modelName = bdroot;

            % Create scope toolbar
            if isempty(obj.pFig.UserData) % Toolbar got cleared
                if isempty(obj.pSimulinkUIToolbar) % Toolbar was never created
                    t = findall(obj.pFig,'Type','uitoolbar');
                    if isempty(t)
                        t = uitoolbar(obj.pFig);
                    end
                    obj.pSimulinkUIToolbar = driving.internal.SimulinkUIToolbar(...
                            'Toolbar', t,...
                            'ModelName', modelName, ...
                            'BlockName', obj.pBlockName);
                else % Make sure that the toolbar is registered to model events
                    registerToModelButtonEvents(obj.pSimulinkUIToolbar);
                end
                userData.SimulinkUIToolbar = obj.pSimulinkUIToolbar;
                obj.pFig.UserData  = userData;
            else
                obj.pSimulinkUIToolbar = obj.pFig.UserData.SimulinkUIToolbar;
                registerToModelButtonEvents(obj.pSimulinkUIToolbar);
            end
            
            % Plot coverage areas
            if obj.HasVision
                plotCoverageAreas(obj, 'visionDetectionGenerator');
            end
            if obj.HasRadar
                plotCoverageAreas(obj, 'radarDetectionGenerator');
            end
            
            % Turn off the legend if it was off earlier
            if ~obj.pIsLegendOn
                legend(obj.pFig.CurrentAxes,'off')
            end
            
            % Bring the figure to front, set it to visible, but prevent
            % others from plotting to it
            isDirty = get_param(bdroot,'Dirty');
            set_param(obj.pBlockName,'UserData',obj.pFig)
            set_param(obj.pBlockName,'ModelCloseFcn','helperCloseAll(gcb)');
            set_param(obj.pBlockName,'OpenFcn','helperOpenFcn');
            set_param(bdroot,'Dirty',isDirty);
            set(obj.pFig,'Visible','on','HandleVisibility','off');
        end

        function releaseImpl(obj)
            % Release resources, such as file handles
            if ~isempty(obj.pFig.UserData)
                modelName = bdroot;
                isFastRestart = strcmp(get_param(modelName,'FastRestart'),'on');
                if isFastRestart %In fast restart mode, SimulationStatus is still 'running' at the end
                    setStoppedIcon(obj.pFig.UserData.SimulinkUIToolbar);
                else
                    update(obj.pFig.UserData.SimulinkUIToolbar);
                end
                release(obj.pFig.UserData.SimulinkUIToolbar);
            end
            dirtyFlag = get_param(bdroot,'Dirty');
            set_param(obj.pBlockName,'OpenFcn','');
            set_param(bdroot,'Dirty',dirtyFlag);
        end

        function flag = isInactivePropertyImpl(obj,prop)
            % Return false if property is visible based on object 
            % configuration, for the command line and System block dialog
            flag = false;
            if ~obj.HasTracks && (strcmpi(prop, 'TrackPositionSelector') || strcmpi(prop, 'TrackVelocitySelector') || strcmpi(prop, 'HasMIO') )
                flag = true;
            end
        end
        
        function s = saveObjectImpl(obj)
            s = saveObjectImpl@matlab.System(obj);
            
            if isLocked(obj)
                s.pFig = obj.pFig;
                s.pBEP = obj.pBEP;
                s.pActorPlotter         = obj.pActorPlotter;
                s.pVisionPlotter        = obj.pVisionPlotter;
                s.pRadarPlotter         = obj.pRadarPlotter;
                s.pTrackPlotter         = obj.pTrackPlotter;
                s.pMIOPlotter           = obj.pMIOPlotter;
                s.pActorProfile         = obj.pActorProfile;
                s.pLaneBoundaryPlotter  = obj.pLaneBoundaryPlotter;
                s.pIsLegendOn           = obj.pIsLegendOn;
                s.pSimulinkUIToolbar    = saveobj(obj.pSimulinkUIToolbar);
            end
        end
        
        function loadObjectImpl(obj,s,wasLocked)
            if wasLocked
                obj.pFig = s.pFig;
                obj.pBEP = s.pBEP;
                obj.pActorPlotter           = s.pActorPlotter;
                obj.pVisionPlotter          = s.pVisionPlotter;
                obj.pRadarPlotter           = s.pRadarPlotter;
                obj.pTrackPlotter           = s.pTrackPlotter;
                obj.pMIOPlotter             = s.pMIOPlotter;
                obj.pActorProfile           = s.pActorProfile;
                obj.pLaneBoundaryPlotter    = s.pLaneBoundaryPlotter;
                obj.pIsLegendOn             = s.pIsLegendOn;
                obj.pSimulinkUIToolbar      = loadobj(s.pSimulinkUIToolbar);
                
                s = rmfield(s,{'pFig','pBEP','pActorPlotter','pVisionPlotter', ...
                    'pRadarPlotter','pTrackPlotter','pActorProfile',...
                    'pLaneBoundaryPlotter','pMIOPlotter','pSimulinkUIToolbar'});
            end
            loadObjectImpl@matlab.System(obj,s,wasLocked);
        end
        
        function stepImpl(obj,varargin)
            %Update the Simulink control toolbar
            update(obj.pFig.UserData.SimulinkUIToolbar);
            
            % Update the bird's-eye plot if it is visible
            if strcmp(get(obj.pFig,'Visible'),'on')
                idx = 1;

                if obj.HasActors
                    actors = varargin{idx};
                    idx = idx+1;

                    plotActors(obj, actors);
                end

                if obj.HasVision
                    dets = varargin{idx};
                    idx = idx+1;

                    plotDetections(obj.pVisionPlotter, dets);
                end

                if obj.HasRadar
                    dets = varargin{idx};
                    idx = idx+1;

                    plotDetections(obj.pRadarPlotter, dets);
                end

                if obj.HasTracks
                    tracks = varargin{idx};
                    plotTracks(obj, tracks);
                    idx = idx+1;
                    if obj.HasMIO
                        mio = varargin{idx};
                        plotMIO(obj,tracks,mio);
                        idx = idx+1;
                    end
                end
                
                if obj.HasRoads
                    rbEgo = varargin{idx};
                    plotLanes(obj,rbEgo);
                end
            end
        end
    end
    
    methods(Access=private)
        function plotActors(obj, actorsBus)
            if (isnumeric(actorsBus) && actorsBus == 0)
                % Input is disconnected, so return
                return
            end

            numActors = actorsBus.NumActors;
            actorPoses = actorsBus.Actors(1:numActors);
            actorProfile = obj.pActorProfile;
            
            numActors = numActors + 1; % To add the ego vehicle
            pos = NaN(numActors,2);
            yaw = NaN(numActors,1);
            
            % Add ego vehicle at origin
            pos(1,:) = [0,0];
            yaw(1)   = 0;
            
            % Add all other vehicles
            for m = 2:numActors
                pos(m,:) = actorPoses(m-1).Position(1:2);
                yaw(m)   = actorPoses(m-1).Yaw;
            end
            plotOutline(obj.pActorPlotter,pos,yaw, ...
                actorProfile.Length*ones(numActors,1), ...
                actorProfile.Width*ones(numActors,1), ...
                'OriginOffset',ones(numActors,1)*actorProfile.OriginOffset);
        end
        
        function plotTracks(obj, tracks)
            if (isnumeric(tracks) && tracks == 0)
                % Input is disconnected, so return
                return
            end
            trs = tracks.Tracks(1:tracks.NumTracks);
            trPos = getTrackPositions(trs, obj.TrackPositionSelector);
            trVel = getTrackVelocities(trs, obj.TrackVelocitySelector);
            plotTrack(obj.pTrackPlotter, trPos, trVel);
        end
        
        function plotMIO(obj,tracks,mio)
            if (isnumeric(tracks) && tracks == 0)
                % Input is disconnected, so return
                return
            end
            if mio>0 && mio<=tracks.NumTracks
                trs = tracks.Tracks(mio);
                trPos = getTrackPositions(trs, obj.TrackPositionSelector);
                plotTrack(obj.pMIOPlotter, trPos);
            else
                plotTrack(obj.pMIOPlotter, nan(1,2));
            end
        end
        
        function plotCoverageAreas(obj, sensorType)
            % Plots coverage areas for all sensors in the current block
            % diagram of sensor type, sensorType.
            %
            % Example:
            %   % Plot coverage areas for all radar sensors in current model
            %   plotCoverageAreas(obj, 'radarDetectionGenerator');
            
            switch sensorType
                case 'radarDetectionGenerator'
                    sensorStr = 'Radar';
                    sensorClr = 'red';
                case 'visionDetectionGenerator'
                    sensorStr = 'Vision';
                    sensorClr = 'blue';
            end
            
            % If plotter already exists, return
            if ~isempty(findPlotter(obj.pBEP,'DisplayName',[sensorStr ' Coverage Area']))
                return
            end
            
            % Find all sensorDetectionGenerator blocks
            currMdl = bdroot;
            sensorBlks = findAllSensorBlocks(currMdl, sensorType);
            
            for m = 1:numel(sensorBlks)
                thisSensor = sensorBlks{m};
                
                % Load all model references on this block's path
                loadedModels = driving.internal.SimulinkUtilities.loadModels(thisSensor);
                clnUp = onCleanup(@()driving.internal.SimulinkUtilities.closeModels(loadedModels));
                
                fov = evalinGlobalScope(currMdl, [sensorType '.fieldOfView(''' thisSensor ''');']);
                maxRng = eval(get_param(thisSensor,'MaxRange'));
                yaw = eval(get_param(thisSensor,'Yaw'));
                sensorLoc = eval(get_param(thisSensor,'SensorLocation'));
                
                if m == 1
                    plotter = coverageAreaPlotter(obj.pBEP,'DisplayName',...
                        [sensorStr ' Coverage Area'],'FaceColor',sensorClr, ...
                        'EdgeColor', sensorClr);
                else
                    plotter = coverageAreaPlotter(obj.pBEP,'FaceColor',sensorClr, ...
                        'EdgeColor', sensorClr);
                end
                plotCoverageArea(plotter,sensorLoc,maxRng,yaw,fov(1));
                
                % Close all referenced models that were loaded
                delete(clnUp);
            end
        end
        
        function plotLanes(obj, rbEgo)
            if (isnumeric(rbEgo) && isscalar(rbEgo) && rbEgo == 0)
                % Input is disconnected, so return
                return
            end
            plotLaneBoundary(obj.pLaneBoundaryPlotter, {rbEgo});
        end
    end
    
    % Simulink interface
    methods(Access=protected)
        function str = getIconImpl(~)
            str = sprintf('Bird''s-Eye\nPlot');
        end
        
        function num = getNumInputsImpl(obj)
            num = 0;
            
            if obj.HasActors
                num = num+1;
            end
            if obj.HasVision
                num = num+1;
            end
            if obj.HasRadar
                num = num+1;
            end
            if obj.HasTracks
                num = num+1;
                if obj.HasMIO
                    num = num+1;
                end
            end
            if obj.HasRoads
                num = num+1;
            end
        end
        function varargout = getInputNamesImpl(obj)
            varargout = {};
            
            if obj.HasActors
                varargout = {varargout{:} 'Actors'}; %#ok<CCAT>
            end
            if obj.HasVision
                varargout = {varargout{:} 'Vision'}; %#ok<CCAT>
            end
            if obj.HasRadar
                varargout = {varargout{:} 'Radar'}; %#ok<CCAT>
            end
            if obj.HasTracks
                varargout = {varargout{:} 'Tracks'}; %#ok<CCAT>
                if obj.HasMIO
                    varargout = {varargout{:} 'MIO'}; %#ok<CCAT>
                end
            end
            if obj.HasRoads
                varargout = {varargout{:} 'Roads'}; %#ok<CCAT>
            end
        end
    end

    methods(Access = protected, Static)
        function header = getHeaderImpl
            % Define header panel for System block dialog
            header = matlab.system.display.Header(...
                'Title', 'BirdsEyePlot',...
                'Text', getHeaderText());
        end

        function simMode = getSimulateUsingImpl
            % Return only allowed simulation mode in System block dialog
            simMode = 'Interpreted execution';
        end

        function flag = showSimulateUsingImpl
            % Return false if simulation mode hidden in System block dialog
            flag = false;
        end
    end
end

function plotDetections(plotter, dets)
if (isnumeric(dets) && dets == 0)
    % Input is disconnected, so return
    return
end
numDets = dets.NumDetections;
if numDets>0
    detPos = NaN(numDets,3);
    for m = 1:numDets
        posMeas = driving.internal.parseDetectionForInitFcn(dets.Detections(m), 'helperBirdsEyePlot', 'double');
        detPos(m,:) = posMeas;
    end
    plotDetection(plotter,detPos);
end
end

function blks = findAllSensorBlocks(mdl, sensorType)
blks = find_system(mdl,'BlockType','MATLABSystem','System',sensorType);

% Check for model reference blocks
refBlks = find_system(mdl,'BlockType','ModelReference');
for m = 1:numel(refBlks)
    thisPath = refBlks{m};
    if ~strcmpi(get_param(thisPath,'ProtectedModel'),'on')
        thisMdl = get_param(thisPath,'ModelName');
        wasLoaded = ~bdIsLoaded(thisMdl);
        if wasLoaded
            load_system(thisMdl);
        end
        theseBlks = findAllSensorBlocks(thisMdl, sensorType);
        if wasLoaded
            close_system(thisMdl, 0);
        end
        blks = {blks{:} theseBlks{:}}; %#ok<CCAT>
    end
end
end


function str = getHeaderText
str = sprintf([...
    'The Birds Eye Plot block creates and maintains a bird''s-eye plot. ',...
    'You can enable the display of actors, vision detections, radar ',...
    'detections, tracks and road boundaries.\n\n',...
    'When the vision and radar detections input ports are enabled, the ',...
    'block will automatically create the coverage areas ',...
    'and plotters for all the visionDetectionGenerator and ',...
    'radarDetectionGenerator blocks in the model (not including ',...
    'protected model references).\n\n',...
    'You must comment this block out when using ''Rapid Accelerator'' mode.']);
end