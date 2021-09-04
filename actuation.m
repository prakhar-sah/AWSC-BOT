classdef actuation
    methods(Static)
       
        function actuator(rpml, rpmr) 
            r = raspi;
            configurePin(r,23,'DigitalOutput');
            configurePin(r,24,'DigitalOutput');
            configurePin(r,25,'PWM');
            configurePin(r,22,'DigitalOutput');
            configurePin(r,27,'DigitalOutput');
            configurePin(r,17,'PWM');
            writePWMDutyCycle(r, 25, 1);
            writePWMFrequency(r, 25, 100);
            writePWMDutyCycle(r, 17, 1);
            writePWMFrequency(r, 17, 100);
            %while true 
                writeDigitalPin(r,23,0);
                writeDigitalPin(r,24,1);
                writeDigitalPin(r,22,0);
                writeDigitalPin(r,27,1);
                writePWMDutyCycle(r, 25, rpmr);
                writePWMDutyCycle(r, 17, rpml);
            %end
                %writePWMDutyCycle(r, 25, 0);
                %writePWMDutyCycle(r, 17, 0);
        end
    end
end