%helperLidarRegistrationLocalizer Localize using Lidar data
%   
%   This is an example helper class that is subject to change or removal in
%   future releases.
%
%   localizer = helperLidarRegistrationLocalizer() creates a lidar
%   localizer object.
%
%   localizer = helperLidarRegistrationLocalizer(Name, Value) specifies
%   additional name value pair arguments as described below:
%
%   'InitialPoseTransform'    Rigid transformation representing the initial
%                             pose.
%                           
%                             Default: rigid3d (identity transformation)
%
%   'LidarToVehicleTransform' Rigid transformation specified as a rigid3d
%                             object representing the transformation from
%                             the origin of the lidar sensor frame to the
%                             vehicle coordinates (center of the vehicle,
%                             on the ground). This is only used for display
%                             purposes.
%
%   'ProcessFcnHandle'        Function handle specifying how to process a
%                             point cloud prior to localization. Function
%                             handle must be of the form:
%                             ptCloud = fcnHandle(ptCloud, name, value,...)
%   
%                             Default: @helperProcessPointCloud
%
%   'ProcessFcnArguments'     Struct specifying name-value pair arguments
%                             to be passed to ProcessFcnHandle.
%                   
%                             Default: []
%
%   'RegisterFcnHandle'       Function handle specifying registration
%                             algorithm to be used for localization. 
%                             Function handle must be of the form: 
%                             tform = fcnHandle(moving, fixed, name, value,...)
%
%                             Default: @pcregistericp
%
%   'RegisterFcnArguments'    Struct specifying name-value pair arguments
%                             to be passed to RegisterFcnHandle.
%
%                             Default: struct('Metric', 'pointToPlane', ...
%                                         'MaxIterations', 40);
%
%   'DisplayPreferences'      Struct specifying display preferences with
%                             fields and values described below:
%                             View                - '2D' or '3D'
%                             VehicleDimensions   - [length, width, height]
%                             VehicleColor        - [r, g, b]
%                             Limits              - [xmin, xmax, ymin, ymax, zmin, zmax]
%
%                             Default: struct(...
%                                         'View',              '2D',
%                                         'VehicleDimensions', [3.864 1.96 1.286], ... % hatchback
%                                         'VehicleColor',      [0.85 0.325 0.098], ...
%                                         'Limits',            'auto');  
%
%   helperLidarRegistrationLocalizer properties:
%   InitialPoseTransform    - Initial pose transformation (read-only)
%   LidarToVehicleTransform - Transformation from lidar to vehicle coordinates (read-only)
%   ProcessFcnHandle        - Function handle to point cloud processing function
%   ProcessFcnArguments     - Name-value arguments struct for processing function
%   RegisterFcnHandle       - Function handle to registration function
%   RegisterFcnArguments    - Name-value pair struct for registration function
%
%   helperLidarRegistrationLocalizer methods:
%   localize        - Localize lidar frame
%   updateDisplay   - Update localization display
%   reset           - Reset localizer
%   
%   Example
%   -------
%   % Create a velodyne file reader object
%   veloReader = velodyneFileReader('lidarData_ConstructionRoad.pcap', 'HDL32E');
%
%   % Create a localizer object
%   lidarLocalizer = helperLidarRegistrationLocalizer;
%
%   skipFrames = 5;
%   numFrames  = 200;
%
%   % Loop through frames and localize
%   for n = 1 : skipFrames : numFrames
%       ptCloud = readFrame(veloReader, n);
%
%       currPose = localize(lidarLocalizer, ptCloud);
%
%       updateDisplay(lidarLocalizer);
%       drawnow update
%   end
%
%   
%   See also pcregistericp, pcregisterndt, pcregistercorr, pointCloud,
%       rigid3d, pcviewset.

% Copyright 2020 The MathWorks, Inc.

classdef helperLidarRegistrationLocalizer < handle
    
    properties (SetAccess = protected)
        %InitialPoseTransform
        %   Rigid transformation specified as a rigid3d object representing
        %   the initial pose. Use this to specify the transformation to the
        %   world frame.
        %   
        %   Default: rigid3d
        InitialPoseTransform(1,1)       rigid3d
        
        %LidarToVehicleTransform
        %   Rigid transformation specified as a rigid3d object representing
        %   the transformation from the origin of the lidar sensor frame to
        %   the vehicle coordinates (center of the vehicle, on the ground).
        %   This is only used for display purposes.
        LidarToVehicleTransform(1,1)    rigid3d
        
        %DisplayPreferences
        %   Struct specifying display preferences with fields and values
        %   described below:
        %   View              - '2D' (default) or '3D'
        %   VehicleDimensions - [length, width, height] (default: [3.864 1.96 1.286])
        %   VehicleColor      - [r, g, b] (default: [0.85 0.325 0.098])
        %   Limits            - 'auto' (default) or [xmin, xmax, ymin, ymax, zmin, zmax]
        %   Location          - 'auto' (default) or position input for
        %                       movegui like 'east', 'north'.
        DisplayPreferences(1,1)         struct
    end
    
    properties (Access = public)
        %ProcessFcnHandle
        %   Function handle specifying how to process a point cloud prior
        %   to localization. The function handle must be of the form:
        %       ptCloud = fcnHandle(ptCloud, name, value, ...)
        %
        %   Default: @helperProcessPointCloud
        ProcessFcnHandle(1,1)
        
        %ProcessFcnArguments
        %   Struct specifying name-value pair arguments to be passed to
        %   ProcessFcnHandle.
        %
        %   Default: []
        ProcessFcnArguments(1,1)    struct
        
        %RegisterFcnHandle
        %   Function handle specifying registration algorithm to be used
        %   for localization. The function handle must be of the form:
        %       tform = fcnHandle(moving, fixed, name, value, ...)
        %
        %   Default: @pcregistericp
        RegisterFcnHandle(1,1)
        
        %RegisterFcnArguments
        %   Struct specifying name-value pair arguments to be passed to
        %   RegisterFcnHandle.
        %
        %   Default: struct('Metric', 'pointToPlane', 'MaxIterations', 40)
        RegisterFcnArguments(1,1)   struct
    end
    
    properties (SetAccess = protected)
        %Map
        %   Accumulated point cloud map, as a pointCloud object.
        Map             pointCloud
        
        %PrevPointCloud
        %   Previous pointCloud, corresponding to the last frame.
        PrevPointCloud  pointCloud
        
        %RelPose
        %   Relative pose estimate from last invocation of localize,
        %   specified as a rigid3d object.
        RelPose         rigid3d
        
        %CurrPose
        %   Current pose estimate from last invocation of localize,
        %   specified as a rigid3d object.
        CurrPose        rigid3d
        
        %Axes
        %   Handle to axes display.
        Axes            matlab.graphics.axis.Axes
    end
    
    properties (Access = protected)
        %Player
        %   Handle to pcplayer object used to display accumulated point
        %   cloud map.
        Player
        
        %PathScatter
        %   Handle to scatter3 object used to display estimate of
        %   localization path.
        PathScatter
    end
    
    methods
        %------------------------------------------------------------------
        function this = helperLidarRegistrationLocalizer(params)
            
            arguments
                params.InitialPoseTransform(1,1)    rigid3d         = rigid3d;
                params.LidarToVehicleTransform(1,1) rigid3d         = rigid3d;
                params.ProcessFcnHandle(1,1)        function_handle = @helperProcessPointCloud;         
                params.ProcessFcnArguments(1,1)     struct          = struct;
                params.RegisterFcnHandle(1,1)       function_handle = @pcregistericp;
                params.RegisterFcnArguments(1,1)    struct          = struct('Metric', 'pointToPlane', 'MaxIterations', 40);
                params.DisplayPreferences(1,1)      struct          = struct(...
                                                                            'View',                 '2D', ...
                                                                            'VehicleDimensions',    [3.864 1.96 1.286], ...
                                                                            'VehicleColor',         [0.85 0.325 0.098], ...
                                                                            'Limits',               'auto', ...
                                                                            'Location',             'auto'); 
            end
            
            this.InitialPoseTransform       = params.InitialPoseTransform;
            this.LidarToVehicleTransform    = params.LidarToVehicleTransform;
            this.ProcessFcnHandle           = params.ProcessFcnHandle;
            this.ProcessFcnArguments        = params.ProcessFcnArguments;
            this.RegisterFcnHandle          = params.RegisterFcnHandle;
            this.RegisterFcnArguments       = params.RegisterFcnArguments;
            this.DisplayPreferences         = params.DisplayPreferences;
            
            reset(this);
        end
        
        %------------------------------------------------------------------
        function [currPose, relPose] = localize(this, ptCloud, params)
            %localize localize lidar point cloud
            %   localize a lidar point cloud using registration.
            %   currPose = localize(localizer, ptCloud) localizes the point
            %   cloud ptCloud and returns the current pose transformation.
            %
            %   [...,relPose] = localize(...) additionally returns the
            %   relative pose transformation since the last invocation of
            %   localize.
            %
            %   currPose = localize(..., Name, Value) specifies additional
            %   name-value pair arguments as described below:
            %
            %   'InitialTransform'  Initial transform estimate to use for
            %                       registration, specified as a rigid3d
            %                       object or a string 'constvel' or
            %                       'none'.
            %
            %                       Default: 'none'
            %
            %   See also rigid3d.
            arguments
                this(1,1)               helperLidarRegistrationLocalizer
                ptCloud(1,1)            pointCloud
                
                params.InitialTransform = "none";
            end
            
            initTform = iValidateInitialTransform(params.InitialTransform, this.RelPose);
            
            % Process point cloud
            processFcnArgs = iStructToNameValueArgs(this.ProcessFcnArguments);
            ptCloud = this.ProcessFcnHandle(ptCloud, processFcnArgs{:});
            
            if isempty(this.PrevPointCloud)
                currPose = this.CurrPose;
                relPose  = rigid3d;
                
                this.Map = pctransform(ptCloud, currPose);
                this.PrevPointCloud = ptCloud;
                return;
            end
            
            % Register point cloud
            registerFcnArgs = iStructToNameValueArgs(this.RegisterFcnArguments);
            hasInit = ~isempty(initTform);
            if hasInit
                relPose = this.RegisterFcnHandle(ptCloud, this.PrevPointCloud, ...
                    'InitialTransform', initTform, registerFcnArgs{:});
            else
                relPose = this.RegisterFcnHandle(ptCloud, this.PrevPointCloud, ...
                    registerFcnArgs{:});
            end
            
            %TODO Restrict rotation to XY Plane only and provide a flag
            relPose.Translation(3) = 0;
            
            currPose = this.CurrPose;
            height = currPose.Translation(3);
            currPose = rigid3d( relPose.T * currPose.T );
            currPose.Translation(3) = height;
            
            % Update map
            this.Map = pccat([this.Map pctransform(ptCloud, currPose)]);
            
            this.CurrPose       = currPose;
            this.RelPose        = relPose;
            this.PrevPointCloud = ptCloud;
        end
        
        %------------------------------------------------------------------
        function isDisplayOpen = updateDisplay(this, closeDisplay)
            %updateDisplay Update display of localizer
            %   isDisplayOpen = updateDisplay(localizer, closeDisplay)
            %   updates the display of localizer. If closeDisplay is true,
            %   the displays are shut down. isDisplayOpen returns a logical
            %   scalar specifying whether the display is open or closed.
            %
            %   See also pcplayer, showShape.
            
            arguments
                this(1,1)           helperLidarRegistrationLocalizer
                closeDisplay(1,1)   logical                             = false;
            end
            
            if closeDisplay
                closeFigureDisplay(this);
                isDisplayOpen = false;
            end
            
            if isempty(this.Axes)
                setupDisplay(this);
            end
            
            displayPrefs = this.DisplayPreferences;
            
            % Update point cloud map display
            view(this.Player, this.Map);
            
            % Set view angle
            if displayPrefs.View(1)=='2'
                view(this.Axes, 2);
            end
            
            % Convert current absolute pose of sensor to absolute pose of
            % vehicle.
            lidarToVehicleTform = this.LidarToVehicleTransform;
            currPose            = this.CurrPose;
            currVehiclePose = rigid3d( lidarToVehicleTform.T * currPose.T );
            
            % Draw localization estimate
            helperDrawVehicle(this.Axes, currVehiclePose, ...
                displayPrefs.VehicleDimensions, 'Color', displayPrefs.VehicleColor);
            
            % Draw localization trajectory
            scatterHandle = this.PathScatter;
            translation   = this.CurrPose.Translation;
            scatterHandle.XData(end+1) = translation(1);
            scatterHandle.YData(end+1) = translation(2);
            scatterHandle.ZData(end+1) = translation(3);
        end
        
        %------------------------------------------------------------------
        function costmap = createCostmap(this, mapSize, spatialLimits)
            %createCostmap Create a 2D costmap from the map
            %   costmap = createCostmap(localizer, mapSize, spatialLimits)
            %   creates a costmap from the accumulated point cloud map. The
            %   size and spatial limits of the costmap are determined by
            %   mapSize and spatialLimits. mapSize is specified as a 1-by-2
            %   vector of [xlen, ylen]. spatialLimits is specified as a
            %   3-by-2 array of the form [xmin,xmax; ymin,ymax; zmin,zmax].
            %
            %   See also createCostmap.
            
            arguments
                this
                mapSize(1,2)        double {mustBeNumeric, mustBeFinite, ...
                                            mustBeInteger, mustBePositive}
                spatialLimits(3,2)  double {mustBeNumeric}                              
            end
            
            if isempty(this.Map) || this.Map.Count==0
                costmap = ones(mapSize);
                return;
            end
            % Bin points
            numBins = [mapSize, 1];
            binIndices = pcbin(this.Map, numBins, spatialLimits);
            
            % Create an occupancy grid
            maxDensity = 100;
            costmap = cellfun(@numel, binIndices);
            costmap = min(costmap, maxDensity);
            costmap = costmap ./ maxDensity;
        end
        
        %------------------------------------------------------------------
        function reset(this)
            %reset Reset lidar localizer
            %   reset(localizer) resets the localizer and clears internal
            %   state. The next call to localize will begin localization.
            
            this.Map            = pointCloud.empty;
            this.PrevPointCloud = pointCloud.empty;
            this.CurrPose       = this.InitialPoseTransform;
            this.RelPose        = rigid3d.empty;
            this.Axes           = matlab.graphics.axis.Axes.empty;
        end
        
        %------------------------------------------------------------------
        function delete(this)
            closeFigureDisplay(this);
        end
    end
    
    methods (Access = protected)
        %------------------------------------------------------------------
        function setupDisplay(this)
            
            displayPrefs = this.DisplayPreferences;
            
            % Create pcplayer
            [xlimits, ylimits, zlimits] = iGetPlayerLimits(displayPrefs.Limits, this.InitialPoseTransform.Translation);
            this.Player = pcplayer(xlimits, ylimits, zlimits);
            this.Axes = this.Player.Axes;
            
            % Adjust view angle
            if displayPrefs.View(1)=='2'
                view(this.Axes, 2);
            end
            
            % Create scatter display for path
            hold(this.Axes, 'on')
            markerSize = 6;
            this.PathScatter = scatter3(this.Axes, NaN, NaN, NaN, ...
                markerSize, displayPrefs.VehicleColor, 'filled');
            hold(this.Axes, 'off')
            
            % Adjust location
            if ~strcmp(displayPrefs.Location, 'auto')
                figureHandle = this.Player.Axes.Parent;
                movegui(figureHandle, displayPrefs.Location);
            end
        end
        
        %------------------------------------------------------------------
        function closeFigureDisplay(this)
            if ~isempty(this.Axes) && isvalid(this.Axes)
                figureHandle = ancestor(this.Axes, 'figure');
                if ~isempty(figureHandle) && isvalid(figureHandle)
                    close(figureHandle);
                end
            end
        end
    end
end

%--------------------------------------------------------------------------
function out = iStructToNameValueArgs(s)
out = reshape( [fieldnames(s), struct2cell(s)]', 1, [] );
end

%--------------------------------------------------------------------------
function tformOut = iValidateInitialTransform(tformIn, prevTform)

validateattributes(tformIn, {'rigid3d','char','string'}, {}, 'localize', 'InitialTransform');
if isa(tformIn, 'rigid3d')
    validateattributes(tformIn, {'rigid3d'}, {'scalar'}, 'localize', 'InitialTransform');
    tformOut = tformIn;
else
    tformIn = validatestring(tformIn, {'constvel', 'none'}, 'localize', 'InitialTransform');
    if tformIn(1) == 'c'
        tformOut = prevTform;
    else
        tformOut = rigid3d.empty;
    end
end
end

%--------------------------------------------------------------------------
function [xlims, ylims, zlims] = iGetPlayerLimits(limits, trans)
if isnumeric(limits)
    validateattributes(limits, {'numeric'}, {'numel', 6, 'finite', 'real'}, mfilename, 'limits');
    xlims = [limits(1) limits(2)];
    ylims = [limits(3) limits(4)];
    zlims = [limits(5) limits(6)];
else
    validatestring(limits, {'auto'}, mfilename, 'limits');
    zone = 50;
    xlims = trans(1) + [-zone zone];
    ylims = trans(2) + [-zone zone];
    zlims = trans(3) + [-zone zone];
end
end