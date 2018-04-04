classdef helperUIConnector < driving.connector.Connector
    
    %---------------------------------------------------------
    % Properties to store internal state of the external tool
    %---------------------------------------------------------
    properties (Access = private)
        
        customUI % holds Custom UI.
        
        updateGTLDisplay = true; % Flag to distinguish the caller of
        % slider value change event. The callers are:
        % (1) frameChangeListener called by
        %     groundTruthLabeler.
        % (2) mouse movement of slider.
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
        function this = helperUIConnector()
            % Load custom data here in the constructor:
            this.customUI = helperCustomUI('distanceData.mat');
        end
    end
    
    methods
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
            if ~endReached(this)
                % Convert 'time' to index in the data to be plotted. ceil
                % it to ensure that it is integral, as it is used to index
                % into the data array.
                index = ceil(this.CurrentTime*this.customUI.lengthOfData()/this.TimeVector(end));
                this.customUI.updateUI(index);
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
            this.customUI.updateUI(1);
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
    end
    methods (Access = private)
        
        %------------------------------------------------------------------
        function flag = endReached(this)
            flag = (this.CurrentTime >= this.TimeVector(end));
        end
        
    end
end
