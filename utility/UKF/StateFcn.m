function x_new = StateFcn(x, dT, u)
%stateTransition Propagate the vehicle forward.

    % Convert velocity samples to pose samples
    thetas = x(3);
    x_new(:,1) = x(1) + u(1).*cos(thetas)*dT;
    x_new(:,2) = x(2) + u(1).*sin(thetas)*dT;
    x_new(:,3) = thetas + u(2)*dT;
    
end
