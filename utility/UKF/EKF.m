classdef EKF<handle
    %UKF Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        x
        P
        n_x        % state dimension                     
        v
        Trace
    end
    
    methods
        
        function obj = EKF(x0,P0)
            %UKF Unscendent kalman filter for unycicle            
            obj.n_x = 3;  % State vector dimension (x,y,theta,v)
            obj.x = x0(1:3);   % Initial state of the system
            obj.P = P0(1:3,1:3);   % Initial state of the system
            obj.Trace = obj.x(:);
        end
        
        % Prediction Predicts sigma points, the state, and the state covariance
        % matrix
        % u is the input at the system
        % dT Time between k and k+1 in s        
        function [] = Prediction(obj,u,dt,cov_u)
             
            % syms x dt v a theta omega y
            % xn = x + (dt*v + a*dt^2/2)*cos(theta+omega*dt/2)
            % yn = y + (dt*v + a*dt^2/2)*sin(theta+omega*dt/2)
            % thetan = theta + omega*dt
            % vn = v + a*dt
            
            % Extract the state
            x = obj.x(1);
            y = obj.x(2);
            theta = obj.x(3);
            v = u(1);
            omega = u(2);
            a = 0;
            
            % Predict the state 
            xn = x + (dt*v + a*dt^2/2)*cos(theta+omega*dt/2);
            yn = y + (dt*v + a*dt^2/2)*sin(theta+omega*dt/2);
            thetan = theta + omega*dt;
            
            

            % Linearizzazione A
            A = zeros(3,3);
            % 1-st ROW
            A(1,1) = 1; % diff(xn,x)
            A(1,3) = 0; % diff(xn,y)
            A(1,3) = -sin(theta + (dt*omega)/2)*((a*dt^2)/2 + v*dt); % diff(xn,theta)
           
            
            % 2-st ROW
            A(2,1) = 0; % diff(yn,x)
            A(2,2) = 1; % diff(yn,y)
            A(2,3) = cos(theta + (dt*omega)/2)*((a*dt^2)/2 + v*dt); % diff(yn,theta)
           
            
            % 3-rd ROW
            A(3,1) = 0; % diff(thetan,x)
            A(3,3) = 0; % diff(thetan,y)
            A(3,3) = 1; % diff(thetan,theta)
           
            
            % Linearizzazione input
            J= zeros(3,2);
            % 1-st row 
            J(1,1) = dt*cos(theta + (dt*omega)/2); % diff(xn,v)
            J(1,2) = -(dt*sin(theta + (dt*omega)/2)*((a*dt^2)/2 + v*dt))/2; % diff(xn,omega)
            
            
            % 2-st row 
            J(2,1) =  dt*sin(theta + (dt*omega)/2); % diff(yn,v)
            J(2,2) = (dt*cos(theta + (dt*omega)/2)*((a*dt^2)/2 + v*dt))/2; % diff(yn,omega)
            
            
            % 3-st row 
            J(3,1) = 0; % diff(thetan,v)
            J(3,2) = dt; % diff(thetan,omega)
            
                        
            % new state
            obj.x = [xn;yn;thetan];
            
            R = J*cov_u*J';
            
            obj.P = A*obj.P*A' + R;
            obj.v = v;
            obj.Trace = [obj.Trace, obj.x]; 
        end
        
        % Updates the state and the state covariance matrix using RFID
        % measurements
        function [] = UpdateRFID(obj,z,R,RFID)
            n_z = numel(z);
            
            px    = obj.x(1);  % Extract te value
            py    = obj.x(2);
            yaw   = obj.x(3);
            nu_v  = obj.v;            
            
%             syms px py yaw nu_v x_rfid y_rfid
%             v = nu_v*[cos(yaw); sin(yaw)];
%             dx = x_rfid - px;
%             dy = y_rfid - py;
%             normaliz = sqrt(dx*dx + dy*dy);
%             z_i = (v(1)*dx + v(2)*dy)/normaliz;
            
            % Transform sigma points into measurement space            
            z_pred = zeros(n_z,1);
            H = eye(n_z,3);    
            for i=1:n_z
                x_rfid = RFID(1,i);
                y_rfid = RFID(2,i);
                z_pred(i) = -(nu_v*cos(yaw)*(px - x_rfid) + nu_v*sin(yaw)*(py - y_rfid))/((px - x_rfid)^2 + (py - y_rfid)^2)^(1/2);
                %diff(z_i,px)
                H(1,1) = ((2*px - 2*x_rfid)*(nu_v*cos(yaw)*(px - x_rfid) + nu_v*sin(yaw)*(py - y_rfid)))/(2*((px - x_rfid)^2 + (py - y_rfid)^2)^(3/2)) - (nu_v*cos(yaw))/((px - x_rfid)^2 + (py - y_rfid)^2)^(1/2);
                %diff(z_i,py)
                H(1,2) = ((2*py - 2*y_rfid)*(nu_v*cos(yaw)*(px - x_rfid) + nu_v*sin(yaw)*(py - y_rfid)))/(2*((px - x_rfid)^2 + (py - y_rfid)^2)^(3/2)) - (nu_v*sin(yaw))/((px - x_rfid)^2 + (py - y_rfid)^2)^(1/2); 
                %diff(z_i,yaw)
                H(1,3) = -(nu_v*cos(yaw)*(py - y_rfid) - nu_v*sin(yaw)*(px - x_rfid))/((px - x_rfid)^2 + (py - y_rfid)^2)^(1/2); 
            end
            
            y = z - z_pred;    % innovation        
            S = R + H*obj.P*H'; % innovation covariance
            K = obj.P*H'/S;     % kalman gain
                        
            % Update state and covariance
            obj.x = obj.x + K*y;     % state update
            obj.P = obj.P - K*H*obj.P;  % covariance update
        end
        
               
        function draw(obj)
            %drawCarBot Routine to draw the car-like robot
            
            if ~(obj.plot_flag)
               return 
            end

            obj.HTrajectory.XData = obj.Trace(1,:);
            obj.HTrajectory.YData = obj.Trace(2,:);
            
            obj.HCenter.XData = obj.x(1);
            obj.HCenter.YData = obj.x(2);
            
            
            
            
             % Plot xy covariance
            [x_el,y_el]=error_ellipse(obj.P(1:2,1:2),obj.x(1:2),0.95);                         
            obj.HPosCov.XData = x_el;
            obj.HPosCov.YData = y_el;
            
            % Plot angle covariance
            L = 0.3;
            ang_cov = 3*sqrt(obj.P(3,3));
            n = ceil(ang_cov/0.05);
            angles = obj.x(3) + ang_cov*linspace(-1,1,2*n);
            obj.HAngleCov.XData = obj.x(1)+[0,cos(angles),0]*L;
            obj.HAngleCov.YData = obj.x(2)+[0,sin(angles),0]*L;
        end
    end
end

% 
% %%%% SYMBOLICAL COMPUTATION
% syms cx cxy cy r11 r21 r22
% H = eye(2,2)
% R = [r11, r21; r21, r22];   % optimize R in order to best lower P
% P = [cx, cxy; cxy, cy];
% S = R + H*P*H';
% K = P*H'/S;
% nP = P - K*H*P;   