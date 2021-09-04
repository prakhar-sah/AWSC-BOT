classdef wall_follow
    methods (Static)
        function wallFollow = follow_wall(xtmin1)
        app = run_app;
        user_stop = app.userStop;
        xm = [];
        ym = [];
        xt = [xtmin1(1), xtmin1(2), xtmin1(3)];
        ltickso = xtmin1(4);
        rtickso = xtmin1(5);
        while not(user_stop || (xt(3)-xtmin1(3) >= 2*pi))
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
            xm = [xm; x(1)];
            ym = [ym; x(2)];
        end
        k = boundary(xm,ym,0);
        map = [xm(k),ym(k)];
        wallFollow = map;
        end
    end
end

            
                
    