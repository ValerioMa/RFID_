classdef UKF<handle
    %UKF Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        x
        P
        n_x        % state dimension
        Xsig_pred  % predicted sigma points
        weights    % weights of sigma points
        n_aug      % augmented state dimension
        lambda     % sigma point spreading parameters
        n_sig      % sigma points dimension
        
    end
    methods(Static)
        % Generate sigma points
        % x: state vector
        % P: covariance matrix
        % lambda: sigma points spreading parameter
        % n_sig: sigma points dimensino
        function Xsig = GenerateSigmaPoints(x,P, lambda, n_sig)
            n = numel(x);
            
            Xsig = zeros(n,n_sig); % sigma point matrix
            
            
            % Quare root of P
            A = P^(1/2);
            
            Xsig(:,1) = x;
            
            lambda_plue_n_x_sqrt = sqrt(lambda + n);
            for i=1:n
                Xsig( :, i + 1 )     = x + lambda_plue_n_x_sqrt * A(:,i);
                Xsig( :, i + 1 + n ) = x - lambda_plue_n_x_sqrt * A(:,i);
            end
        end
        
        % Predits sigma points.
        % Xsig : Sigma points to predict.
        % delta_t : Time between k and k+1 in s
        % n_x : State dimension.
        % n_sig : Sigma points dimension.
        % nu_am : Process noise standard deviation longitudinal acceleration in m/s^2
        % nu_yawdd : Process noise standard deviation yaw acceleration in rad/s^2        
        function Xsig_pred= PredictSigmaPoints(Xsig, delta_t, n_x, n_sig)
            Xsig_pred = zeros(n_x,n_sig);
            
            % Predict sigma point
            for i=1:n_sig
                p_x = Xsig(1,i);  % Extract te value
                p_y = Xsig(2,i);
                yaw = Xsig(3,i);
                nu_v = Xsig(4,i);
                nu_yawd = Xsig(5,i);
                
                px_p  = p_x + nu_v*delta_t*cos(yaw);
                py_p  = p_y + nu_v*delta_t*sin(yaw);
                yaw_p = yaw + nu_yawd*delta_t;
                
                % Store the predicted sigma point into right column
                Xsig_pred(1,i) = px_p;
                Xsig_pred(2,i) = py_p;
                Xsig_pred(3,i) = yaw_p;
                Xsig_pred(4,i) = nu_v;
                Xsig_pred(5,i) = nu_yawd;
            end
        end
        
        % Normalized the component `index` of the vector `vector` to be inside [-M_PI, M_PI] interval.
        function value = NormalizeAngleOnComponent(value)
            while (value> pi)
                value = value - 2*pi;
            end
            while (value< -pi)
                value = value + 2*pi;
            end
        end
    end
    
    methods
        
        function obj = UKF(x0,P0)
            %UKF Unscendent kalman filter for unycicle            
            obj.n_x = 3;  % State vector dimension (x,y,theta)
            obj.x = x0;   % Initial state of the system
            obj.P = P0;   % Initial state of the system
            
            
            obj.lambda = 3 - obj.n_x;      % lambda sigma points
            obj.n_aug = obj.n_x + 2;       % Augmented state dimension
            obj.n_sig = 2 * obj.n_aug + 1; % Sigma points dimension
            
            % Initialize weights
            obj.weights = zeros(obj.n_sig,1);
            obj.weights(:) = 0.5/(obj.lambda+obj.n_aug);
            obj.weights(1) = obj.lambda/(obj.lambda+obj.n_aug);
            
        end
        
         
        % Prediction Predicts sigma points, the state, and the state covariance
        % matrix
        % u is the input at the system
        % dT Time between k and k+1 in s
        
        function [] = Prediction(obj,u,dT,cov_u)
             
            % Initialize sigma points
            x_aug = zeros(obj.n_aug,1);  % augmented mean vector
            x_aug(1:3) = obj.x;
            x_aug(4:5) = u;
            P_aug      = zeros(obj.n_aug,obj.n_aug); %Augmented covariance
            P_aug(1:3,1:3) = obj.P;
            P_aug(4:5,4:5) = cov_u;
            
            % Generate sigma points
            Xsig_aug = obj.GenerateSigmaPoints(x_aug,P_aug, obj.lambda, obj.n_sig);
            
            % Predict Sigma points
            obj.Xsig_pred = obj.PredictSigmaPoints(Xsig_aug,dT,obj.n_x,obj.n_sig);

            % Predicted mean and covariance
            obj.x = obj.Xsig_pred(1:3,:)*obj.weights;  % mean
            P_tmp = zeros(obj.n_x,obj.n_x);
            for i=1:obj.n_sig
                x_diff = obj.Xsig_pred(1:3,i) - obj.x;
                x_diff(3) = obj.NormalizeAngleOnComponent(x_diff(3));
                P_tmp = P_tmp + obj.weights(i) * (x_diff * x_diff');
            end
            obj.P = P_tmp;   % covariance
        end
        
        % Updates the state and the state covariance matrix using RFID
        % measurements
        function [] = UpdateRFID(obj,z,R,RFID)
            %  1. Predict measurement
            n_z = numel(z);
            Zsig = zeros(n_z,obj.n_sig);            
            z = z(:);
%             v = mean(obj.Xsig_pred(4,:));
%             for i=1:n_z
%                 if v < abs(z(i))
%                     z(i) = sign(z(i))*v;
%                 end
%             end
            % Transform sigma points into measurement space
            for i =1:obj.n_sig
                p_x     = obj.Xsig_pred(1,i);  % Extract te value
                p_y     = obj.Xsig_pred(2,i);
                yaw     = obj.Xsig_pred(3,i);
                nu_v    = obj.Xsig_pred(4,i);
                %nu_yawd = Xsig(5,i);
                
                v = nu_v*[cos(yaw); sin(yaw)];
                
                % measurement model
                for k=1:n_z
                    x_rfid = RFID(1,k);
                    y_rfid = RFID(2,k);
                    
                    dx = x_rfid - p_x;
                    dy = y_rfid - p_y;
                    normaliz = sqrt(dx*dx + dy*dy);
                    if(normaliz<1e-1)
                        return;
                        %Zsig(k,i) = Nan;
                    else
                        Zsig(k,i) = (v(1)*dx + v(2)*dy)/normaliz;
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


