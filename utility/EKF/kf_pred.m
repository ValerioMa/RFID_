function [s_new,P_new] = kf_pred(s,P,u,cov_u)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here



% EQUAZIONE DELLA DINAMICA UNICICLO
dinamica = @(s,u) s + [u(1)*cos(s(3));u(2)*sin(s(3));u(2)];

% stato_new = A*stato_old + spostamento dato da controlli
A = eye(3,3); 

% PROPAGAZIONE INCERTEZZA
Q = prop_cov(s,cov_u);

s_new = dinamica(s,u);

P_new = A*P*A' + Q;

end




function [Q] = prop_cov(s,cov_u)
    J = [cos(s(3)), 0; sin(s(3)), 0; 0, 1];
    
    Q = J*cov_u*J';
end