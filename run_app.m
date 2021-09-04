classdef run_app < matlab.apps.AppBase

    % Properties that correspond to app components
    properties (Access = public)
        UIFigure                   matlab.ui.Figure
        Image                      matlab.ui.control.Image
        BuildMapButton             matlab.ui.control.Button
        STOPButton                 matlab.ui.control.Button
        DeployForCollectionButton  matlab.ui.control.Button
        userStop
    end

    % Callbacks that handle component events
    methods (Access = private)

        % Button pushed function: BuildMapButton
        function map = BuildMapButtonPushed(app, event)
            app.userStop = 0;
            i = first_wall.find_first_wall([0,0,0]);
            map = wall_follow.follow_wall(i);
            save('map.mat','map');
        end

        % Button pushed function: DeployForCollectionButton
        function DeployForCollectionButtonPushed(app, event)
            
            app.userStop = 0;
            
            %startup 
            %dont forget to kill & reboot stateCount process on raspberry pi before
            %deploying for collection
            
            map = load('map.mat','map');

            odometryModel = odometryMotionModel;
            odometryModel.Noise = [0.2 0.2 0.2 0.2]; %alpha1,alpha2,alpha3,alpha4
            rangeFinderModel = likelihoodFieldSensorModel;
            rangeFinderModel.SensorLimits = [0.04 1];
            rangeFinderModel.Map = map;
            rangeFinderModel.SensorPose = [0.385 0 0]; %pose of range sensor relative to the bot's pose
            f = sensor_model.fwall();
            laserSub = f(2);

            amcl = monteCarloLocalization;
            amcl.UseLidarScan = false;
            amcl.MotionModel = odometryModel;
            amcl.SensorModel = rangeFinderModel;
            amcl.UpdateThresholds = [0.1,0.1,0.1];
            amcl.ResamplingInterval = 1;
            amcl.GlobalLocalization = true;
            amcl.ParticleLimits = [500 50000];
            odompose = [0, 0, 0];
            ltickso = 0;
            rtickso = 0; 
            numUpdates = 60;
            i=0;
            while i<numUpdates || app.userStop == 0
                % Create ranges and angles object from laserSub to pass to the AMCL object.
                angles = linspace(-0.042*pi, 0.042*pi);
                ranges = laserSub*ones(1,numel(angles));
                % Update estimated robot's pose and covariance using new odometry and
                % sensor readings.
                [isUpdated,estimatedPose, estimatedCovariance] = amcl(odompose, ranges, angles);
                xt = estimatedPose;
                % Drive robot to next pose depending on motion defined (ut) || run
                % actuation file (wallfollow.py) written in python
                        if sensor_model.bumpLeft || sensor_model.bumpRight
                            actuation.actuator(0, 0.3); %experiment
                            x = motion_model.motion_model_odometry(xt, ltickso, rtickso);
                            xt = [x(1), x(2), x(3)];
                            ltickso = x(4);
                            rtickso = x(5);
                        elseif not(sensor_model.rwall)
                            actuation.actuator(1, 0.3); %experiment
                            x = motion_model.motion_model_odometry(xt, ltickso, rtickso);
                            xt = [x(1), x(2), x(3)];
                            ltickso = x(4);
                            rtickso = x(5);
                        else
                            actuation.actuator(0.3,0.3);
                            x = motion_model.motion_model_odometry(xt, ltickso, rtickso);
                            xt = [x(1), x(2), x(3)];
                            ltickso = x(4);
                            rtickso = x(5);
                        end
                % Drive robot to next pose depending on motion defined (ut) || run
                % actuation file (wallfollow.py) written in python
                odompose = xt;
                mclp = odompose;
                mclc = estimatedCovariance;
                if isUpdated 
                    i = i + 1;
                end
            end
            actuation.actuator(0,0); %kill the velocity
            
            %navigation model
            %in the conditions oh while loops, in norm (xt-xo) etc, xt,xo or xg are not poses
            %but only (x,y) values.
            
            odometryModel = odometryMotionModel;
                odometryModel.Noise = [0.2 0.2 0.2 0.2]; %alpha1,alpha2,alpha3,alpha4
                rangeFinderModel = likelihoodFieldSensorModel;
                rangeFinderModel.SensorLimits = [0.04 1];
                rangeFinderModel.Map = map;
                rangeFinderModel.SensorPose = [0.385 0 0]; %pose of range sensor relative to the bot's pose

                amcl = monteCarloLocalization;
                amcl.UseLidarScan = false;
                amcl.MotionModel = odometryModel;
                amcl.SensorModel = rangeFinderModel;
                amcl.UpdateThresholds = [0.1,0.1,0.1];
                amcl.ResamplingInterval = 1;
                amcl.ParticleLimits = [500 5000];
                amcl.GlobalLocalization = false;
                amcl.InitialPose = mclp;
                amcl.InitialCovariance = mclc;
            
                delta = 0.06; %6cm
                epsilon = 0.01; %1cm
                xt = mclp;
                
            while app.userStop == 0
                xg %from map
                %xt from monteCarlo (keeps changing continuosly)
                xo %from sensor data + function to convert data into pose of obstacles
                while norm(xt-xo) > delta + epsilon && norm(xt-xg) > epsilon
                UGTG = actions.Ugtg(xt,xg);
                actuation.actuator(UGTG(2),UGTG(3));
                x = MonteCarloLocalization.mc_localization(xt, amcl, ltickso, rtickso);
                xt = [x(1),x(2),x(3)];
                ltickso = x(4);
                rtickso = x(5);
                amcl = x(6);
                end
                UGTG = actions.Ugtg(xt,xg);
                UFWC = actions.Ufwc(xt,xo);
                UFWCC = actions.Ufwcc(xt,xo);
                UAO = actions.Uao(xt,xo);
                if UGTG(1)'*UFWC(1) > 0 && norm(xt-xo) > delta - epsilon && norm(xt-xo) < delta + epsilon
                    dt = norm(xt-xg);
                    while not(norm(xt-xg) < dt && UAO(1)'*UGTG(1) > 0 && norm(xt-xo) < delta - epsilon)
                    UFWC = actions.Ufwc(xt,xo);
                    actuation.actuator(UFWC(2),UFWC(3));
                    x = MonteCarloLocalization.mc_localization(xt, amcl, ltickso, rtickso);
                    xt = [x(1),x(2),x(3)];
                    ltickso = x(4);
                    rtickso = x(5);
                    amcl = x(6);
                    UGTG = actions.Ugtg(xt,xg);
                    UAO = actions.Uao(xt,xo);
                    end
                    if norm(xt-xo) < delta - epsilon
                        while not(norm(xt-xo) > delta - epsilon && norm(xt-xo) < delta + epsilon)
                        UAO = actions.Uao(xt,xo);
                        actuation.actuator(UAO(2),UAO(3));
                        x = MonteCarloLocalization.mc_localization(xt, amcl, ltickso, rtickso);
                        xt = [x(1),x(2),x(3)];
                        ltickso = x(4);
                        rtickso = x(5);
                        amcl = x(6);
                        end
                    end
                elseif UGTG(1)'*UFWCC(1) > 0 && norm(xt-xo) > delta - epsilon && norm(xt-xo) < delta + epsilon
                    dt = norm(xt-xg);
                    while not(norm(xt-xg) < dt && UAO(1)'*UGTG(1) > 0 && norm(xt-xo) < delta - epsilon)
                    UFWCC = actions.Ufwcc(xt,xo);
                    actuation.actuator(UFWCC(2),UFWCC(3));
                    x = MonteCarloLocalization.mc_localization(xt, amcl, ltickso, rtickso);
                    xt = [x(1),x(2),x(3)];
                    ltickso = x(4);
                    rtickso = x(5);
                    amcl = x(6);
                    UGTG = actions.Ugtg(xt,xg);
                    UAO = actions.Uao(xt,xo);
                    end
                    if norm(xt-xo) < delta - epsilon
                        while not(norm(xt-xo) > delta - epsilon && norm(xt-xo) < delta + epsilon)
                        UAO = actions.Uao(xt,xo);
                        actuation.actuator(UAO(2),UAO(3));
                        x = MonteCarloLocalization.mc_localization(xt, amcl, ltickso, rtickso);
                        xt = [x(1),x(2),x(3)];
                        ltickso = x(4);
                        rtickso = x(5);
                        amcl = x(6);
                        end
                    end
                elseif norm(xt-xg) <= epsilon
                    %update xg
                end
            end
            actuation.actuator(0,0); %kill the velocity
        end
        
        
        % Button pushed function: STOPButton
        function STOPButtonPushed(app, event)
            app.userStop = 1;
        end
    end

    % Component initialization
    methods (Access = private)

        % Create UIFigure and components
        function createComponents(app)

            % Create UIFigure and hide until all components are created
            app.UIFigure = uifigure('Visible', 'off');
            app.UIFigure.Position = [100 100 563 394];
            app.UIFigure.Name = 'MATLAB App';

            % Create DeployForCollectionButton
            app.DeployForCollectionButton = uibutton(app.UIFigure, 'push');
            app.DeployForCollectionButton.ButtonPushedFcn = createCallbackFcn(app, @DeployForCollectionButtonPushed, true);
            app.DeployForCollectionButton.BackgroundColor = [0 0 0];
            app.DeployForCollectionButton.FontName = 'Gill Sans MT';
            app.DeployForCollectionButton.FontSize = 16;
            app.DeployForCollectionButton.FontColor = [1 1 1];
            app.DeployForCollectionButton.Position = [166 118 253 44];
            app.DeployForCollectionButton.Text = 'Deploy For Collection';

            % Create STOPButton
            app.STOPButton = uibutton(app.UIFigure, 'push');
            app.STOPButton.ButtonPushedFcn = createCallbackFcn(app, @STOPButtonPushed, true);
            app.STOPButton.BackgroundColor = [0.6353 0.0784 0.1843];
            app.STOPButton.FontName = 'Gill Sans Ultra Bold Condensed';
            app.STOPButton.FontSize = 20;
            app.STOPButton.FontWeight = 'bold';
            app.STOPButton.FontColor = [1 1 1];
            app.STOPButton.Position = [72 44 442 44];
            app.STOPButton.Text = 'STOP';

            % Create BuildMapButton
            app.BuildMapButton = uibutton(app.UIFigure, 'push');
            app.BuildMapButton.ButtonPushedFcn = createCallbackFcn(app, @BuildMapButtonPushed, true);
            app.BuildMapButton.BackgroundColor = [0 0 0];
            app.BuildMapButton.FontName = 'Gill Sans MT';
            app.BuildMapButton.FontSize = 16;
            app.BuildMapButton.FontColor = [1 1 1];
            app.BuildMapButton.Position = [229 190 125 44];
            app.BuildMapButton.Text = 'Build Map';

            % Create Image
            app.Image = uiimage(app.UIFigure);
            app.Image.Position = [242 269 100 100];
            app.Image.ImageSource = 'kisspng-turtle-computer-icons-symbol-5aefbc27599402.4752950915256607113669.png';

            % Show the figure after all components are created
            app.UIFigure.Visible = 'on';
        end
    end

    % App creation and deletion
    methods (Access = public)

        % Construct app
        function app = run_app

            % Create UIFigure and components
            createComponents(app)

            % Register the app with App Designer
            registerApp(app, app.UIFigure)

            if nargout == 0
                clear app
            end
        end

        % Code that executes before app deletion
        function delete(app)

            % Delete UIFigure when app is deleted
            delete(app.UIFigure)
        end
    end
end