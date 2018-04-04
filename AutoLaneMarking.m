% This class defines an Automatic lane detection algorithm. To use this
% algorithm from within the groundTruthLabeler app, save this file as
% follows.
%
%  Create a +vision/+labeler folder within a folder that is already
%  on the MATLAB path. For example, if the folder /local/MyProject is on
%  the MATLAB path, then create a +vision/+labeler folder hierarchy as
%  follows:
%
%    mkdir /local/MyProject +vision/+labeler
%
%  Saving the file to the package directory is required to use this custom
%  algorithm from within the groundTruthLabeler app. You can add a folder
%  to the path using the ADDPATH function.
%
%  Save this algorithm class in the folder created in step 1. Refresh the
%  algorithm list from within the groundTruthLabeler app to start using
%  this custom algorithm.

classdef AutoLaneMarking < vision.labeler.AutomationAlgorithm
    
    %----------------------------------------------------------------------
    % Step 1: Define required properties describing the algorithm. This
    %         includes Name, Description, and UserDirections.
    properties(Constant)
        
        % Name: Give a name for your algorithm.
        Name = 'Auto Lane Detection';
        
        % Description: Provide a one-line description for your algorithm.
        Description = 'Automatically detect lane like features';
        
        % UserDirections: Provide a set of directions that are displayed
        %                 when this algorithm is invoked. The directions
        %                 are to be provided as a cell array of character
        %                 vectors, with each element of the cell array
        %                 representing a step in the list of directions.
        UserDirections = {...
            'Load a MonoCamera configuration object from the workspace using the settings panel',...
            'Specify additional parameters in the settings panel',...
            'Run the algorithm',...
            'Manually inspect and modify results if needed'};
    end
    
    %---------------------------------------------------------------------
    % Step 2: Define properties to be used during the algorithm. These are
    % user-defined properties that can be defined to manage algorithm
    % execution.
    properties
        %MonoCamera
        %  The monoCamera object associated with this video
        MonoCamera          = [];
        %MonoCameraVarname
        %  The workspace variable name of the monoCamera object
        MonoCameraVarname   = '';
        %BirdsEyeConfig
        %  The birdsEyeView object needed to create the bird's-eye view
        BirdsEyeConfig      = [];
        %MaxNumLanes
        %  The maximum number of lanes the algorithm tries to annotate
        MaxNumLanes         = 2;
        %ROI
        %  The region of interest around the vehicle used to search for
        %  lanes
        ROI            = [3, 30, -3, 3];
        %LaneMaskSensitivity
        %  The sensitivity parameter used in the segmentLaneMarkerRidge function
        LaneMaskSensitivity = 0.25;
        %LaneBoundaryWidth
        %  The lane boundary width, used in findParabolicLaneBoundaries
        LaneBoundaryWidth   = 0.6;
        %XPoints
        %  The x-axis points along which to mark the lane boundaries
        XPoints             = [3 3.5 4 4.5 5 6 7 10 30];
    end
    
    %----------------------------------------------------------------------
    % Step 3: Define methods used for setting up the algorithm.
    methods
        % a) Specify the label definitions for which your algorithm is
        %    valid by defining a checkLabelDefinition method.
        %
        %    For more help,
        %    >> help vision.labeler.AutomationAlgorithm.checkLabelDefinition
        %
        function TF = checkLabelDefinition(~, labelDef)
            % Lane detection only works with Line type labels
            TF = labelDef.Type == labelType.Line;
        end
        
        % b) Specify the conditions that need to be met for algorithm
        %    set up to be complete by defining the checkSetup method. This
        %    method must return false till the user has completed all the
        %    steps required to setup the algorithm.
        %
        %    For more help,
        %    >> help vision.labeler.AutomationAlgorithm.checkSetup
        %
        function TF = checkSetup(algObj, ~)
            % This is the only required input
            TF = ~isempty(algObj.MonoCamera);
        end
        
        % c) Optionally, specify what settings the algorithm requires by
        %    implementing the settingsDialog method. This method is invoked
        %    when the user clicks the Settings button.
        %
        %    For more help,
        %    >> help vision.labeler.AutomationAlgorithm.checkSetup
        %
        function settingsDialog(algObj)
            
            % Input descriptions
            prompt={...
                'Enter the MonoCamera variable name',...
                'ROI',...
                'Maximum number of Lanes',...
                'Lane mask sensitivity',...
                'Lane boundary width',...
                'X Coordinates',...
                };
            defaultAnswer={...
                algObj.MonoCameraVarname,...
                num2str(algObj.ROI),...
                num2str(algObj.MaxNumLanes),...
                num2str(algObj.LaneMaskSensitivity),...
                num2str(algObj.LaneBoundaryWidth),...
                num2str(algObj.XPoints),...
                };
            
            name='Settings for lane detection';
            numLines=1;
            
            allValid = false;
            while(~allValid)  % Repeat till all inputs pass validation
                
                % Create the settings dialog
                options.Resize='on';
                options.WindowStyle='normal';
                options.Interpreter='none';
                answer = inputdlg(prompt,name,numLines,defaultAnswer,options);
                
                if isempty(answer)
                    % Cancel
                    break;
                end
                
                try
                    % Parse and validate inputs
                    algObj.MonoCameraVarname = answer{1};
                    algObj.MonoCamera = evalin('base',algObj.MonoCameraVarname);
                    validateattributes(algObj.MonoCamera,...
                        {'monoCamera'},{'scalar'});
                    
                    algObj.ROI = eval(['[,' answer{2}, ']']);
                    validateattributes(algObj.ROI,...
                        {'double'},{'numel',4});
                    
                    % Define bird's-eye-view transformation parameters
                    outImageSize = [NaN, 250]; % output image width in pixels
                    
                    algObj.BirdsEyeConfig      = birdsEyeView(algObj.MonoCamera, algObj.ROI, outImageSize);
                    algObj.MaxNumLanes         = str2double(answer{3});
                    algObj.LaneMaskSensitivity = str2double(answer{4});
                    algObj.LaneBoundaryWidth   = str2double(answer{5});
                    algObj.XPoints             = eval(['[,' answer{6}, ']']);
                    allValid = true;
                catch ALL
                    waitfor(errordlg(ALL.message,'Invalid settings'));
                end
            end
        end
    end
    
    %----------------------------------------------------------------------
    % Step 4: Specify algorithm execution. This controls what happens when
    %         the user presses RUN. Algorithm execution proceeds by first
    %         executing initialize on the first frame, followed by run on
    %         every frame, and terminate on the last frame.
    methods
        % a) Specify the initialize method to initialize the state of your
        %    algorithm.
        %
        %    For more help,
        %    >> help vision.labeler.AutomationAlgorithm.initialize
        %
        function initialize(~, ~, ~)
        end
        
        % b) Specify the run method to process an image frame and execute
        %    the algorithm. Algorithm execution begins at the first image
        %    frame in the interval and is sequentially invoked till the
        %    last image frame in the interval. Algorithm execution can
        %    produce a set of labels which are to be returned in
        %    autoLabels.
        %
        %    For more help,
        %    >> help vision.labeler.AutomationAlgorithm.run
        %
        function autoLabels = run(algObj, I)
            Ig = rgb2gray(I);
            birdsEyeViewImage = transformImage(algObj.BirdsEyeConfig, Ig);
            birdsEyeViewBW    = segmentLaneMarkerRidge(birdsEyeViewImage, ...
                algObj.BirdsEyeConfig, algObj.LaneBoundaryWidth, ...
                'Sensitivity', algObj.LaneMaskSensitivity);
            
            % Obtain lane candidate points in world coordinates
            [imageX, imageY] = find(birdsEyeViewBW);
            boundaryPointsxy = imageToVehicle(algObj.BirdsEyeConfig, [imageY, imageX]);
            
            % Fit requested number of boundaries to it
            lbs = findParabolicLaneBoundaries(...
                boundaryPointsxy,algObj.LaneBoundaryWidth, ...
                'MaxNumBoundaries',algObj.MaxNumLanes);
            numDetectedLanes = numel(lbs);
            
            % Convert the model to discrete set of points at the specified
            % x-coordinates
            boundaryPoints = cell(1,numDetectedLanes);
            xPoints = algObj.XPoints';
            for ind = 1:numel(lbs)
                yPoints             = lbs(ind).computeBoundaryModel(xPoints);
                boundaryPoints{ind} = vehicleToImage(algObj.MonoCamera, [xPoints, yPoints]);
            end
            
            % Package up the results in a table
            autoLabels = table(...
                boundaryPoints',...
                repmat(labelType.Line, [numDetectedLanes,1]),...
                repmat(algObj.SelectedLabelDefinitions.Name, [numDetectedLanes,1]));
            autoLabels.Properties.VariableNames = {'Position','Type','Name'};
        end
        
        % c) Specify the terminate method to clean up state of the executed
        %    algorithm.
        %
        %    For more help,
        %    >> help vision.labeler.AutomationAlgorithm.terminate
        function terminate(~)
        end
    end
end