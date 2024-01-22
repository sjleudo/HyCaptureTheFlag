function [Boundary_defenders_pos, attackers_pos] = Positions_in_Matrix(MRxp, Robots, NA,ti)
% Rxp is the x position of my robot
% Robots is the opponent team
% Check team    
    fn = fieldnames(Robots);
    Boundary_defenders_pos = []; % We append here the positions of the opponent robots
    attackers_pos = [];
    % If the robot is tagged or has lost tagging ability, it position value
    % is stored as in the opposite half so it is not accounted as an active
    % opponent
    for m=1:NA
        z = fn{1+4*(m-1)};
        tau = fn{2+4*(m-1)};
        q = fn{3+4*(m-1)};
        Boundary_defenders_pos = [Boundary_defenders_pos; -sign(MRxp)*100*((norm(MRxp)>norm(Robots.(z)(ti,1)))*(sign(MRxp*Robots.(z)(ti,1))>0)+...
            (Robots.(tau)(ti)<0)+Robots.(q)(ti))+Robots.(z)(ti,1:2)];
        attackers_pos = [attackers_pos; -sign(MRxp)*100*((norm(MRxp)<norm(Robots.(z)(ti,1)))*(sign(MRxp*Robots.(z)(ti,1))>0)+...
            (Robots.(tau)(ti)<0)+Robots.(q)(ti))+Robots.(z)(ti,1:2)];
    end