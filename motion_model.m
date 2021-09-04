classdef motion_model
    methods(Static)
        
        function xt = motion_model_odometry(xtmin1, ltickso, rtickso)
            r = raspi;
            try
            getFile(r,'/home/pi/Desktop/encoderL.dat','D:\docs\BE Project\Algorithm\Main Program');
            getFile(r,'/home/pi/Desktop/encoderR.dat','D:\docs\BE Project\Algorithm\Main Program');
            lticks = load('encoderL.dat');
            rticks = load('encoderR.dat');
            %disp(lticks);
            %disp(rticks);
            catch
                disp('error in retrieving data from encoder.dat');
                lticks = ltickso;
                rticks = rtickso;
            end
            dl = ((lticks-ltickso)/200)*0.04;
            dr = ((rticks-rtickso)/200)*0.04;
            d = (dl+dr)/2;
            phi = (dr - dl)/L; %L is distance between propellers
            xmin1 = xtmin1(1);
            ymin1 = xtmin1(2);
            thetamin1 = xtmin1(3);
            x = xmin1 + d*cos(phi+thetamin1); %should be d1, for now placing d
            y = ymin1 + d*sin(phi+thetamin1); %should be d1, for now placing d
            theta = phi + thetamin1; %guided by rwall
            %theta = atan2(sin(theta),cos(theta)); need theta to exceed 360
            %deg for mapping
            ltickso = lticks;
            rtickso = rticks;
            xt = [x, y, theta, ltickso, rtickso];
        end
        function xt = motion_model_odometry_1(xtmin1, ltickso, rtickso)
            r = raspi;
            try
            getFile(r,'/home/pi/Desktop/encoderL.dat','D:\docs\BE Project\Algorithm\Main Program');
            getFile(r,'/home/pi/Desktop/encoderR.dat','D:\docs\BE Project\Algorithm\Main Program');
            lticks = load('encoderL.dat');
            rticks = load('encoderR.dat');
            %disp(lticks);
            %disp(rticks);
            catch
                disp('error in retrieving data from encoder.dat');
                lticks = ltickso;
                rticks = rtickso;
            end
            dl = ((lticks-ltickso)/200)*0.04;
            dr = ((rticks-rtickso)/200)*0.04;
            d = (dl+dr)/2;
            phi = (dr - dl)/L; %L is distance between propellers
            xmin1 = xtmin1(1);
            ymin1 = xtmin1(2);
            thetamin1 = xtmin1(3);
            x = xmin1 + d*cos(phi+thetamin1);
            y = ymin1 + d*sin(phi+thetamin1);
            theta = phi + thetamin1; 
            theta = atan2(sin(theta),cos(theta));
            ltickso = lticks;
            rtickso = rticks;
            xt = [x, y, theta, ltickso, rtickso];
        end
    end
end
            