classdef EKF<handle
    %UKF Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        x
        P
        n_x        % state dimension        
        weights    % weights of sigma points                
        
    end
    
    methods
        
        function obj = EKF(x0,P0)
            %UKF Unscendent kalman filter for unycicle
            
            obj.n_x = 3;  % State vector dimension (x,y,theta)
            obj.x = x0;   % Initial state of the system
            obj.P = P0;   % Initial state of the system
                       
            
        end
        
        
        % Normalized the component `index` of the vector `vector` to be inside [-M_PI, M_PI] interval.
        function value = NormalizeAngleOnComponent(obj,value)
            while (value> pi)
                value = value - 2*pi;
            end
            while (value< -pi)
                value = value + 2*pi;
            end
        end
        
        
        % Prediction Predicts sigma points, the state, and the state covariance
        % matrix
        % u is the input at the system
        % dT Time between k and k+1 in s
        
        function [] = Prediction(obj,u,dT,cov_u)
             
            A = zeros(3,2);
            A(1,1) = cos(obj.x(3))*dT;
            A(2,1) = sin(obj.x(3))*dT;
            A(3,2) = 1*dT;
            % Initialize sigma points            
            obj.x = obj.x + A*u(:);
            
            R = A*cov_u*A';
            
            obj.P = obj.P + R;

        end
        
        % Updates the state and the state covariance matrix using RFID
        % measurements
        function [] = UpdateRFID(obj,z,R,RFID)
            %  1. Predict measurement
            n_z = numel(z);
            Zsig = zeros(n_z,obj.n_sig);
            z = z(:);

            % Transform sigma points into measurement space
            z_pred = zeros(n_z,1);
            H = zeros(3,2);
            for i =1:obj.n_sig
                p_x     = obj.x(1);  % Extract te value
                p_y     = obj.x(2);
                yaw     = obj.x(3);
                
                %nu_yawd = Xsig(5,i);
                
                v = nu_v*[cos(yaw); sin(yaw)];
                
                % measurement model
                for k=1:n_z
                    x_rfid = RFID(1,k);
                    y_rfid = RFID(2,k);
                    
                    dx = x_rfid - p_x;
                    dy = y_rfid - p_y;
                    normaliz = sqrt(dx*dx + dy*dy);
                    if(normaliz<1e-4)
                        z_pred(i) = Nan;
                    else
                        z_pred(i) = (v(1)*dx + v(2)*dy)/normaliz;
                    end
                end
            end
            
            
            % mean predicted measurement
            z_pred = zeros(n_z,1);
            for i=1:obj.n_sig
                z_pred = z_pred + obj.weights(i)*Zsig(:,i) ;
            end
            
            % Measurement covariance matrix S
            S = zeros(n_z,n_z);
            for i=1:obj.n_sig
                z_diff = Zsig(:,i) - z_pred;
                S = S + obj.weights(i)*(z_diff*z_diff');
            end
            S = S + R; % add measurement noise covariance matrix
            
            % Update state
            Tc = zeros(obj.n_x,n_z);
            for i=1:obj.n_sig
                z_diff = Zsig(:,i) - z_pred; % residuo
                x_diff = obj.Xsig_pred(1:3,i) - obj.x; % state difference
                Tc = Tc + obj.weights(i)*(x_diff*z_diff');
            end
            
            K = Tc/S; % kalman gain K
            
            z_diff = z - z_pred; % residual
            
            % Update state and covariance
            obj.x = obj.x + K*z_diff;
            obj.P = obj.P - K*S*K';
        end
    end
end


