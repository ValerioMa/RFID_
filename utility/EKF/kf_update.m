function [s,P] = kf_update(s,measure,P,V)

H = eye(3,3);
 
K = P*H'/(H*P*H' + V);

error = measure - H*s;
s = s + K*error;

P = (eye(3,3) - K*H)*P;

end