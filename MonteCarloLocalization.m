classdef MonteCarloLocalization
    methods(Static)
        function xt = mc_localization(xtmin1, amcl, ltickso, rtickso)
            
            %amcl object passed to this function via nav model
            
            f = sensor_model.fwall;
            subUltrasonic = f(2);
            x = motion_model.motion_model_odometry_1(xtmin1, ltickso, rtickso);
            odompose = [x(1),x(2),x(3)];
            ltickso = x(4);
            rtickso = x(5);
            % Create ranges and angles object from laserSub to pass to the AMCL object.
            angles = linspace(-0.042*pi, 0.042*pi);
            ranges = subUltrasonic*ones(1,numel(angles));
            % Update estimated robot's pose and covariance using new odometry and
            % sensor readings.
            [isUpdated,estimatedPose, estimatedCovariance] = amcl(odompose, ranges, angles);
            xt = [estimatedPose, ltickso, rtickso, amcl];

        end
    end
end