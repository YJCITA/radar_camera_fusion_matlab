%% Connecting Lidar Display with groundTruthLabeler
% This example shows how to connect Lidar data visualization tool with
% Ground Truth Labeler app. Note that this example requires use of
% synchronized video and Lidar data. It's specifically designed for
% '01_city_c2s_fcw_10s.mp4' video. To use another set of data,
% the MATLAB code below needs to be modified. To run the example
% invoke the following command:
% 
%   groundTruthLabeler('01_city_c2s_fcw_10s.mp4','ConnectorTargetHandle',@LidarDisplay);
%
% Copyright 2016 The MathWorks, Inc.

classdef LidarDisplay < driving.connector.Connector
    
    %---------------------------------------------------------
    % Properties to store internal state of the external tool
    %---------------------------------------------------------
    properties (Access = private)
        
        hFig    % Figure
        hPlayer % Player for visualizing streaming 3-D Lidar data
        hSlider % Slider
        
        hLeftText  % Text box on the left of the slider.
                   %   This shows the video start time.
        hRightText % Text box on the right of the slider.
                   %   This shows the video end time.
        updateGTLDisplay = true; % Flag to distinguish the caller of
                              % slider value change event. The callers are:
                              % (1) frameChangeListener called by
                              % groundTruthLabeler.
                              % (2) mouse movement of slider.
        
        data % Lidar data loaded from .mat file
    end
    
    methods
        %------------------------------------------------------------------
        % Constructor:
        % A Constructor with no input argument must be defined. This
        % constructor is expected to create an external tool. The external
        % tool can be a custom display or a visualization tool.
        %
        % The Constructor is called by the Ground Truth Labeler app to
        % instantiate the external tool.
        %------------------------------------------------------------------
        function this = LidarDisplay()
            createUI(this);
        end
        
        %------------------------------------------------------------------
        %frameChangeListener Update external tool when a frame changes in the app.
        %   frameChangeListener(connObj) provides an option to
        %   synchronize the external tool with frame changes in the Ground
        %   Truth Labeler app. The app calls this method whenever a new
        %   frame is displayed in the app. This method must be implemented
        %   by the client class.
        %
        %   Input:
        %   ------
        %   connObj   - Connector object.    
        %
        %   See also driving.connector.Connector.dataSourceChangeListener         
        %------------------------------------------------------------------  
        function frameChangeListener(this)
            
            % Move custom slider programmatically
            this.updateGTLDisplay = false;
            if endReached(this)
                this.hSlider.Value=min(this.VideoEndTime,this.hSlider.Max);
                % Value set calls PostSet listener "sliderMoveCallback"
            else
                this.hSlider.Value=max(min(this.CurrentTime, ...
                    this.hSlider.Max), this.hSlider.Min);
                % Value set calls PostSet listener "sliderMoveCallback"
            end
            this.updateGTLDisplay = true;
        end
        
        %------------------------------------------------------------------
        %dataSourceChangeListener Update external tool when a new data 
        %   source is loaded in the app.
        %   dataSourceChangeListener(connObj) provides an option to
        %   update the external tool when a new data source is loaded into
        %   the Ground Truth Labeler app. The app calls this method using
        %   the connObj object. You can optionally use this method to
        %   react to a new data source being connected to the app.
        %
        %   A new data source can be a video, image sequence, or custom
        %   reader. A new data source can be loaded when loading a new
        %   session.
        %
        %   Inputs:
        %   -------
        %   connObj   - Connector object.         
        %
        %   See also driving.connector.Connector.frameChangeListener.  
        %------------------------------------------------------------------  
        function dataSourceChangeListener(this)
            
            % Update states of the custom display
            
            validateInputVideo(this);
            % Warning free value set
            warnState = warning('off', ...
                'MATLAB:hg:uicontrol:ValueMustBeInRange');
            this.hSlider.Min = this.VideoStartTime;
            this.hSlider.Max = this.VideoEndTime;
            this.hSlider.Value = this.hSlider.Min;
            warning(warnState);
            
            this.hLeftText.String=num2str(this.hSlider.Min);
            this.hRightText.String=sprintf('%0.5f',double(this.hSlider.Max));
            
            
            % Create a new label name - must be a valid variable name
            this.LabelName = 'LidarDistance';
            
            % Display Lidar data for first frame
            updateDisplayAtSliderValue(this, this.hSlider.Value);
            
            % Attach listener for slider's movement
            addlistener(this.hSlider,'Value', 'PostSet', ...
                @this.sliderMoveCallback);
        end
        
        %------------------------------------------------------------------
        % closeFig
        % Callback internal to the custom display
        %------------------------------------------------------------------
        function closeFig(this, ~, ~)
            % This display is closed by user; so set flag for GTL
            disconnect(this);
            closereq();
        end
        
        %------------------------------------------------------------------
        %close Close the external tool when the app is closed.
        %   close(connObj) provides an option to close the external
        %   tool when the Ground Truth Labeler closes. The app calls this
        %   method using the connObj object.
        %
        %   Input:
        %   ------
        %   connObj   - Connector object.     
        %
        %   See also driving.connector.Connector.disconnect. 
        %------------------------------------------------------------------
        function close(this)
            % Close this display
            delete(this.hFig);
        end
    end
    methods (Access = private)
        
        %------------------------------------------------------------------
        function createUI(this)
            
            var = load('01_city_c2s_fcw_10s_Lidar.mat');
            this.data = var.LidarPointCloud;
            
            
            % Compute proper x-y limits to ensure Lidar data display is not
            % clipped.
            xyzLimit = getMaxLimit(this);
            xlimits = [xyzLimit.minX xyzLimit.maxX];
            ylimits = [xyzLimit.minY xyzLimit.maxY];
            zlimits = [xyzLimit.minZ xyzLimit.maxZ];
            
            % Create the player to visualize Lidar data
            this.hPlayer = pcplayer(xlimits, ylimits, zlimits);
            
            % Bookkeeping
            hAxes = this.hPlayer.Axes;
            this.hFig = hAxes.Parent;
            rotate3d(this.hFig, 'off');
            this.hFig.Tag = 'LidarDisplay';
            
            % Make room for horizontal slider
            hAxes.Position = [0.1300    0.2100    0.7750    0.7750];
            
            % Render Slider with default settings
            startT = 0;
            endT = 1;
            this.hSlider = uicontrol('Parent',this.hFig, ...
                'Style','slider','Position',[81,54,419,23],...
                'value',0, 'min', startT, 'max', endT);
            
            % Add text boxes to show minimum and maximum slider values
            bgcolor = this.hFig.Color;
            startT_str = num2str(startT);
            endT_str = num2str(endT);
            this.hLeftText = uicontrol('Parent',this.hFig, ...
                'Style','text','Position',[50,54,23,23],...
                'String', startT_str,'BackgroundColor',bgcolor);
            this.hRightText = uicontrol('Parent',this.hFig, ...
                'Style','text','Position',[500,54,63,23],...
                'String',endT_str,'BackgroundColor',bgcolor);
            
            % Add callback to close the display
            set(this.hFig, 'CloseRequestFcn', @this.closeFig);
        end
        
        %------------------------------------------------------------------
        function updateDisplayAtSliderValue(this, val)
            
            idx = sliderValue2LidarDataIndex(this, val);
            view(this.hPlayer, this.data(idx).ptCloud);
        end
        
        %------------------------------------------------------------------
        function sliderMoveCallback(this, ~, ~)
            val = this.hSlider.Value;
            
            % Update Lidar display
            updateDisplayAtSliderValue(this, val);
            
            if this.updateGTLDisplay
                % Update ground truth labeler's video display
                newTime = convertSliderValueToTime(this, val);
                updateLabelerCurrentTime(this, newTime);
            end
        end
        
        %------------------------------------------------------------------
        function  newTime = convertSliderValueToTime(~, sliderValue)
            % Here slider's min value = videoStartTime
            %      slider's max value = videoEndTime
            % That's why slider's value directly represents current time in
            % Ground Truth Labeler's range slider
            newTime =  sliderValue;
        end
        
        %------------------------------------------------------------------
        function flag = endReached(this)
            flag = (this.CurrentTime >= this.TimeVector(end));
        end
        
        %------------------------------------------------------------------
        function xyzLimit = getMaxLimit(this)
            
            LidarData = this.data;
            minX = inf;
            maxX = -inf;
            
            minY = inf;
            maxY = -inf;
            
            minZ = inf;
            maxZ = -inf;
            
            for ii=1:length(LidarData)
                minX = min(minX, LidarData(ii).ptCloud.XLimits(1));
                maxX = max(maxX, LidarData(ii).ptCloud.XLimits(2));
                
                minY = min(minY, LidarData(ii).ptCloud.YLimits(1));
                maxY = max(maxY, LidarData(ii).ptCloud.YLimits(2));
                
                minZ = min(minZ, LidarData(ii).ptCloud.ZLimits(1));
                maxZ = max(maxZ, LidarData(ii).ptCloud.ZLimits(2));
            end
            
            xyzLimit.minX = minX;
            xyzLimit.maxX = maxX;
            
            xyzLimit.minY = minY;
            xyzLimit.maxY = maxY;
            
            xyzLimit.minZ = minZ;
            xyzLimit.maxZ = maxZ;
        end
        
        %------------------------------------------------------------------
        function idx = timeToIndex(this, t)
            
            % idx computation for video displayed on groundTruthLabeler:
            % time: [t1 t2 t3 t4 ...]
            % index:[1  2  3  4 ...]
            % t1<=t<t2 represents frame index=1
            % t2<=t<t3 represents frame index=2
            
            idx = find(this.TimeVector >= t, 1);
            
            numVideoFrames = length(this.TimeVector);
            numLidarFrames = length(this.data);
            if isempty(idx)
                idx = numVideoFrames;
            else
                val = this.TimeVector(idx);
                if (val ~= t)
                    idx = max(1, idx - 1);
                end
            end
            
            % idx computation for Lidar display
            mf = numLidarFrames/numVideoFrames;
            idx = floor(idx * mf);
            
            % Saturate
            if idx <= 0
                idx = 1;
            elseif idx >= numLidarFrames
                idx = numLidarFrames;
            end
        end
        
        %------------------------------------------------------------------
        function idx = sliderValue2LidarDataIndex(this, val)
            
            t = convertSliderValueToTime(this, val);
            idx = timeToIndex(this, t);
            
        end
        
        %------------------------------------------------------------------
        function validateInputVideo(this)
            isValidVideo = ((this.VideoEndTime == 10.2) && ...
                            (length(this.TimeVector) == 204));
            if ~isValidVideo
                error('This example can be run only when ''01_city_c2s_fcw_10s.mp4'' video is loaded on ''groundTruthLabeler''');
            end
        end
    end
end
