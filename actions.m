classdef actions
    methods(Static)
        function rotation_matrix = rotate(theta)
            rotation_matrix = [cos(theta), -sin(theta); sin(theta), cos(theta)];
        end
        function transformation = transform(u,xt)
            t = [1, 0; 0, 1/0.01]; %l = 1cm
            r = rotate(-xt(3));
            wee = t*r*u;
            v = wee(1);
            w = wee(2);
            R = 0.012; %effective differential drive wheel radius of the propellers
            L = 0.073; %distance between propellers
            c = 0.0000067; %constant determined experimentally (dependant on pitch and gear ratio of propeller) 
            vr = (2*v + w*L)/(2*R);
            vl = (2*v - w*L)/(2*R);
            rpmr = round(vr/c);
            rpml = round(vl/c);
            transformation = [rpml, rpmr];
        end
        function UGTG = Ugtg(xt,xg)
            e = [xg(1);xg(2)]-[xt(1);xt(2)];
            v0 = 0.00067; %set velocity limit
            alpha = 1; %determine optimum value through experiment
            e1 = norm(e);
            k = (v0*(1-e1^(-alpha*e1^2)))/e1;
            u = k*e;
            rpm = transform(u, xt);
            %actuation.actuator(rpml, rpmr);
            UGTG = [u,rpm(1),rpm(2)];
        end
          function UAO = Uao(xt,xo)
            e = [xt(1);xt(2)]-[xo(1);xo(2)];
            epsilon = 1;
            q = 1; %determine optimum values of q and epsilon through experiment
            e1 = norm(e);
            k = (1/e1)*(q/((e1^2)+epsilon));
            u = k*e;
            rpm = transform(u, xt);
            %actuation.actuator(rpml, rpmr);
            UAO = [u,rpm(1),rpm(2)];
          end
         function UFWC = Ufwc(xt,xo)
            e = [xt(1);xt(2)]-[xo(1);xo(2)];
            epsilon = 1;
            q = 1; %determine optimum values of q and epsilon through experiment
            e1 = norm(e);
            k = (1/e1)*(q/((e1^2)+epsilon));
            uao = k*e;
            alpha = 1;
            u = alpha*rotate(-pi/2)*uao;
            rpm = transform(u, xt);
            %actuation.actuator(rpml, rpmr);
            UFWC = [u,rpm(1),rpm(2)];
         end
          function UFWCC = Ufwcc(xt,xo)
            e = [xt(1);xt(2)]-[xo(1);xo(2)];
            epsilon = 1;
            q = 1; %determine optimum values of q and epsilon through experiment
            e1 = norm(e);
            k = (1/e1)*(q/((e1^2)+epsilon));
            uao = k*e;
            alpha = 1;
            u = alpha*rotate(pi/2)*uao;
            rpm = transform(u, xt);
            %actuation.actuator(rpml, rpmr);
            UFWCC = [u,rpm(1),rpm(2)];
          end
    end
end