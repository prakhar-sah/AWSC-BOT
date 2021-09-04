classdef first_wall
    methods (Static)
        function firstWall = find_first_wall(xtmin1) 
            app = run_app;
            user_stop = app.userStop;
            foundWall = false;
            xt = xtmin1;
            ltickso = 0;
            rtickso = 0;
            while foundWall == false || user_stop ~= 1
                if sensor_model.bumpLeft || sensor_model.bumpRight || sensor_model.rwall
                    foundWall = true;
                else
                    actuation.actuator(0.3,0.3);
                    x = motion_model.motion_model_odometry(xt, ltickso, rtickso);
                    xt = [x(1),x(2),x(3)];
                    ltickso = x(4);
                    rtickso = x(5);
                end
            end
            firstWall = x;
        end
    end
end
                    
                
            
                
                