classdef sensor_model
    methods(Static)
        function f = fwall
            r = raspi;
            %while true
                distance = system(r,'python /home/pi/Desktop/ultrasonic_sensor.py');
                X = str2double(distance);
                %disp(X);
                if X >=0 && X <= 20
                    front = 1;
                else
                    front = 0;
                end
                f = [front,(X/100)];
            %end
        end
        function left = lwall
            r = raspi;
            configurePin(r,20,'DigitalInput');
            %while true
                left = not(readDigitalPin(r,20));
                if left == 1
                    ldist = 13.5; %make it 7cm
                else
                    ldist = 0;
                end
                %disp(ldist);
            %end
        end
        function right = rwall
            r = raspi;
            configurePin(r,16,'DigitalInput');
            %while true
                right = not(readDigitalPin(r,16));
                if right == 1
                    rdist = 14.5; %make it 7cm
                else
                    rdist = 0;
                end
                %disp(rdist);
            %end
        end
        function bleft = bumpLeft
            r = raspi;
            configurePin(r,26,'DigitalInput');
            %while true
                bleft = not(readDigitalPin(r,26));
                %disp(bleft);
            %end
        end
        function bright = bumpRight
            r = raspi;
            configurePin(r,19,'DigitalInput');
            %while true
                bright = not(readDigitalPin(r,19));
                %disp(bright);
            %end
        end
    end
end