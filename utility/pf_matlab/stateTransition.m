function predictParticles = stateTransition(pf, prevParticles, dT, u) %#ok<INUSL>
%stateTransition Propagate particles based on given motion model.

    thetas = prevParticles(:,3);
    
    w = u(2);
    v = u(1);
    
    l = length(prevParticles);

    % Generate velocity samples
    sdx = 0.05*dT;
    sdtheta = 0.17*dT;
    vh = v*dT;  
    wh = w*dT; 


    % Convert velocity samples to pose samples
    predictParticles(:,1) = prevParticles(:,1) + vh.*cos(thetas) + sdx*randn(l,1);
    predictParticles(:,2) = prevParticles(:,2) + vh.*sin(thetas) + sdx*randn(l,1);
    predictParticles(:,3) = prevParticles(:,3) + wh*dT + sdtheta*randn(l,1);
    predictParticles(:,4) = v;
    
    
end