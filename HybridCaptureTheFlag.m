%--------------------------------------------------------------------------
% Matlab M-file Project: Capture-the-Flag @  Hybrid Systems Laboratory (HSL), 
% Filename: HybridCaptureTheFlag.m
%--------------------------------------------------------------------------
% Project: Hybrid Formulation of Capture the flag - Aquaticus
% Author: Santiago J. Leudo
%--------------------------------------------------------------------------
%--------------------------------------------------------------------------
%   Copyright @ Hybrid Systems Laboratory (HSL),
%   Revision: 0.0.0.1 Date: 11/28/2023 12:44:00

clear all
%close all
clc 


% --------------------------------------------------------------
%%% Initialization
% --------------------------------------------------------------
%%   Paremeters: 
% Dimension of the Playing Field
x_f = [-80 80 80 -80 -80];
y_f = [-40 -40 40 40 -40];
X = [-80 -40 80 40]; % Playing Field
X_B = [-80 -40 0 40]; % Team Blue Region
X_R = [0 -40 80 40]; % Team Red Region

% Flag Region
l=linspace(0,2*pi,200);
flx=sin(l);
fly=cos(l);

gf = 10; % Radius of flag capture region
gc = 10; % Radius of robot tagging region

FB = [-60,0]; % Flag position Team B
FR = [60,0]; % Flag position Team R

b = 3; % Number of robots in Team B
r = 3; % Number of robots in Team R

ip = 25; % Initial distance of robots from the flag

barT = 2; % Timeout after being tagged

% Initial State of Robots
for k=1:b
    z =  sprintf('z_%dB',k);
    tau =  sprintf('tau_%dB',k);
    q =  sprintf('q_%dB',k);
    eta =  sprintf('eta_%dB',k);

    RobotsB.(z)(1,1:3) = [FB(1)+ip*cos(-pi/2+k*pi/(b+1)),...
                      FB(2)+ip*sin(-pi/2+k*pi/(b+1)),...
                      -pi/2+pi*rand(1)];
    
    RobotsB.(tau)(1) = 0; % Starts with tagging ability
    
    RobotsB.(q)(1) = 0; % Starts not tagged
    
    RobotsB.(eta)(1) = 0; % Starts not carrying the flag
end
    RobotsB.mu(1) = 1; % Flag in place
for i=1:r
    z =  sprintf('z_%dR',i);
    tau =  sprintf('tau_%dR',i);
    q =  sprintf('q_%dR',i);
    eta =  sprintf('eta_%dR',i);

    RobotsR.(z)(1,1:3) = [FR(1)+ip*cos(pi/2+i*pi/(r+1)),...
                      FR(2)+ip*sin(pi/2+i*pi/(r+1)),...
                      pi/2+pi*rand(1)];
    
    RobotsR.(tau)(1) = 0; % Starts with tagging ability
    
    RobotsR.(q)(1) = 0; % Starts not tagged
    
    RobotsR.(eta)(1) = 0; % Starts not carrying the flag
end
    RobotsR.mu(1) = 1; % Flag in place
% Simulation Horizon
TSPAN = [0 50];     % Second entry is the maximum amount of seconds allowed
dt = 0.01;
%
% --------------------------------------------------------------
%%% System Evolution 
% --------------------------------------------------------------
for ti=1:TSPAN(2)/dt
    for k=1:b
        z =  sprintf('z_%dB',k);
        tau =  sprintf('tau_%dB',k);
        q =  sprintf('q_%dB',k);
        eta =  sprintf('eta_%dB',k);
        % Jump of logic variables:
        for i=1:r
            zi =  sprintf('z_%dR',i);
            taui =  sprintf('tau_%dR',i);
            qi =  sprintf('q_%dR',i);
            etai =  sprintf('eta_%dR',i); 
            %%%%%%%% Tagging in blue zone
            if pointInRectangle(RobotsB.(z)(ti,1:2),X_B) && RobotsB.(q)(ti) == 0 && RobotsB.(tau)(ti) <= 0 ...
                    && pointInRectangle(RobotsR.(zi)(ti,1:2),X_B) && RobotsR.(qi)(ti) == 0 ...
                      && pointInCircle(RobotsR.(zi)(ti,1:2), RobotsB.(z)(ti,1:2), gc)
    
                RobotsB.(tau)(ti) = Tagging_Ability(RobotsB.(tau)(ti),barT); % Jump in tagging ability 
                RobotsR.(qi)(ti) = 1 - RobotsR.(qi)(ti); % Jump in tagged state
                % Jump if carrying flag
                if RobotsR.(etai)(ti) == 1 && RobotsB.mu(ti) == 0 
                    RobotsB.mu(ti) = 1 - RobotsB.mu(ti); % Jump in flag position state 
                    RobotsR.(etai)(ti) = 1 - RobotsR.(etai)(ti); % Jump in carrying the flag state
                end
                    
            end
            %%%%%%%% Tagged in red zone
            if (pointInRectangle(RobotsB.(z)(ti,1:2),X_R) && RobotsB.(q)(ti) == 0  ...
                    && pointInRectangle(RobotsR.(zi)(ti,1:2),X_R) && RobotsR.(qi)(ti) == 0 && RobotsR.(taui)(ti) <= 0 ...
                      && pointInCircle(RobotsB.(z)(ti,1:2), RobotsR.(zi)(ti,1:2), gc)) ...
               || (not(pointInRectangle(RobotsB.(z)(ti,1:2),X)) && RobotsB.(q)(ti) == 0)  ... % Leaving X
               || (pointInCircle(RobotsB.(z)(ti,1:2),FB,gf) && RobotsB.(q)(ti) == 1)  % Reactivated
                    
                    % Tagged in red zone
                    if (pointInRectangle(RobotsB.(z)(ti,1:2),X_R) && RobotsB.(q)(ti) == 0  ...
                    && pointInRectangle(RobotsR.(zi)(ti,1:2),X_R) && RobotsR.(qi)(ti) == 0 && RobotsR.(taui)(ti) <= 0 ...
                      && pointInCircle(RobotsB.(z)(ti,1:2), RobotsR.(zi)(ti,1:2), gc)) 

                        RobotsR.(taui)(ti) = Tagging_Ability(RobotsR.(taui)(ti),barT); % Jump in tagging ability 
                    end

                    % Either tB gets tagged or leaves X while carrying the flag 
                    if not(pointInCircle(RobotsB.(z)(ti,1:2),FB,gf) && RobotsB.(q)(ti) == 1) ...  %Not being reactivated
                           && RobotsB.(eta)(ti) == 1 && RobotsR.mu(ti) == 0 % Carries the flag
                        RobotsB.(eta)(ti) = 1 - RobotsB.(eta)(ti); % Not carrying flag anymore
                        RobotsR.mu(ti) = 1 - RobotsR.mu(ti); % Flag returned to base
                    end
    
                    RobotsB.(q)(ti) = 1 - RobotsB.(q)(ti); % Jump in tagged state 
            end
            %%%%%%%% First case accounted above: tagged in red zone w flag
            %if  (pointInRectangle(RobotsB.(z)(ti,1:2),X_R) && RobotsB.(q)(ti) == 0 && RobotsB.(eta)(ti) == 1  ...
                 %   && pointInRectangle(RobotsR.(zi)(ti,1:2),X_B) && RobotsR.(qi)(ti) == 0 && RobotsR.(taui)(ti) <= 0 ...
                 %     && pointInCircle(RobotsB.(z)(ti,1:2), RobotsR.(zi)(ti,1:2), gc) && RobotsB.mu(ti) == 0) ||...
             if   (pointInCircle(RobotsB.(z)(ti,1:2),FR,gf) && RobotsB.(q)(ti) == 0 && RobotsB.(eta)(ti) == 0 && RobotsR.mu(ti) == 1) ... % Red flag captured 
               || (pointInCircle(RobotsB.(z)(ti,1:2),FB,gf) && RobotsB.(q)(ti) == 0 && RobotsB.(eta)(ti) == 1 && RobotsR.mu(ti) == 0) % Team B Scored

                RobotsR.mu(ti) = 1 - RobotsR.mu(ti); % Jump in red flag position state (captured or dropped) 
                RobotsB.(eta)(ti) = 1 - RobotsB.(eta)(ti); % Jump in carrying the flag state
            end
            %%%%%%%% First case accounted above: tagged in blue zone w flag
            %if (pointInRectangle(RobotsB.(z)(ti,1:2),X_B) && RobotsB.(q)(ti) == 0 && RobotsB.(tau)(ti) <= 0 ...
             %       && pointInRectangle(RobotsR.(zi)(ti,1:2),X_B) && RobotsR.(qi)(ti) == 0 && RobotsR.(etai)(ti) == 1 ...
              %        && pointInCircle(RobotsR.(zi)(ti,1:2), RobotsB.(z)(ti,1:2), gc) && RobotsB.mu(ti) == 0) ||...
             if (pointInCircle(RobotsR.(zi)(ti,1:2),FB,gf) && RobotsR.(qi)(ti) == 0 && RobotsR.(etai)(ti) == 0 && RobotsB.mu(ti) == 1) ... % Blue flag captured 
               || (pointInCircle(RobotsR.(zi)(ti,1:2),FR,gf) && RobotsR.(qi)(ti) == 0 && RobotsR.(etai)(ti) == 1 && RobotsB.mu(ti) == 0) ... % Team R Scored
               || (not(pointInRectangle(RobotsR.(zi)(ti,1:2),X)) && RobotsR.(qi)(ti) == 0 && RobotsR.(etai)(ti) == 1 && RobotsB.mu(ti) == 0) % Leaving X
                
                 if not(pointInRectangle(RobotsR.(zi)(ti,1:2),X))
                     RobotsR.(qi)(ti) = 1-RobotsR.(qi)(ti) ;
                 end
                RobotsR.(etai)(ti) = 1 - RobotsR.(etai)(ti); % Jump in carrying the flag state
                RobotsB.mu(ti) = 1 - RobotsB.mu(ti); % Jump in blue flag position state (captured or dropped) 
            end
        end
        % Controller
        thetab = Angle_Control(ti,RobotsB,RobotsR,z,tau,q,eta,FB,FR,b,r, dt);
        % Flow of robot:
            RobotsB.(z)(ti+1,1:3) = Robot_Dynamics(RobotsB.(z)(ti,1:3),[50,thetab],dt);  
        % Flow of logic variables:
        RobotsB.(tau)(ti+1) = RobotsB.(tau)(ti) - dt;
        RobotsB.(q)(ti+1) = RobotsB.(q)(ti);
        RobotsB.(eta)(ti+1) = RobotsB.(eta)(ti);
    end
    RobotsB.mu(ti+1) = RobotsB.mu(ti); 
%
% Team Red
%
    for i=1:r
        z =  sprintf('z_%dR',i);
        tau =  sprintf('tau_%dR',i);
        q =  sprintf('q_%dR',i);
        eta =  sprintf('eta_%dR',i);
        % Jump of logic variables:
        for k=1:b
            zk =  sprintf('z_%dB',k);
            tauk =  sprintf('tau_%dB',k);
            qk =  sprintf('q_%dB',k);
            etak =  sprintf('eta_%dB',k); 
            %%%%%%%% Tagging in red zone
%             if pointInRectangle(RobotsR.(z)(ti,1:2),X_R) && RobotsR.(q)(ti) == 0 && RobotsR.(tau)(ti) <= 0 ...
%                     && pointInRectangle(RobotsB.(zk)(ti,1:2),X_R) && RobotsB.(qk)(ti) == 0 ...
%                       && pointInCircle(RobotsB.(zk)(ti,1:2), RobotsR.(z)(ti,1:2), gc)
%                 % Jump if carrying flag
%                 if RobotsB.(etak)(ti) == 1 && RobotsR.mu(ti) == 0 
%                     RobotsR.mu(ti) = 1 - RobotsR.mu(ti); % Jump in flag position state 
%                     RobotsB.(etak)(ti) = 1 - RobotsB.(etak)(ti); % Jump in carrying the flag state
%                 end
%                 RobotsR.(tau)(ti) = Tagging_Ability(RobotsR.(tau)(ti),barT); % Jump in tagging ability 
%                 RobotsB.(qk)(ti) = 1; - RobotsB.(qk)(ti); % Jump in tagged state     
%             end
            %%%%%%%% Tagged in blue zone
            if (pointInRectangle(RobotsR.(z)(ti,1:2),X_B) && RobotsR.(q)(ti) == 0  ...
                    && pointInRectangle(RobotsB.(zk)(ti,1:2),X_B) && RobotsB.(qk)(ti) == 0 && RobotsB.(tauk)(ti) <= 0 ...
                      && pointInCircle(RobotsR.(z)(ti,1:2), RobotsB.(zk)(ti,1:2), gc)) ...
               || (not(pointInRectangle(RobotsR.(z)(ti,1:2),X)) && RobotsR.(q)(ti) == 0)  ... % Leaving X
               || (pointInCircle(RobotsR.(z)(ti,1:2),FR,gf) && RobotsR.(q)(ti) == 1)  % Reactivated
                    
                    % Tagged in blue zone
                    if (pointInRectangle(RobotsR.(z)(ti,1:2),X_B) && RobotsR.(q)(ti) == 0  ...
                    && pointInRectangle(RobotsB.(zk)(ti,1:2),X_B) && RobotsB.(qk)(ti) == 0 && RobotsB.(tauk)(ti) <= 0 ...
                      && pointInCircle(RobotsR.(z)(ti,1:2), RobotsB.(zk)(ti,1:2), gc)) 

                        RobotsB.(tauk)(ti) = Tagging_Ability(RobotsB.(tauk)(ti),barT); % Jump in tagging ability 
                    end

                    % Either tB gets tagged or leaves X while carrying the flag 
                    if not(pointInCircle(RobotsR.(z)(ti,1:2),FR,gf) && RobotsR.(q)(ti) == 1) ...
                           && RobotsR.(eta)(ti) == 1 && RobotsB.mu(ti) == 0 
                        RobotsR.(eta)(ti) = 1 - RobotsR.(eta)(ti); % Not carrying flag anymore
                        RobotsB.mu(ti) = 1 - RobotsB.mu(ti); % Flag returned to base
                    end
    
                    RobotsR.(q)(ti) = 1 - RobotsR.(q)(ti); % Jump in tagged state 
            end
            %%%%%%%% First case accounted above: tagged in blue zone w flag
            %if  (pointInRectangle(RobotsR.(z)(ti,1:2),X_B) && RobotsR.(q)(ti) == 0 && RobotsR.(eta)(ti) == 1  ...
                 %   && pointInRectangle(RobotsB.(zk)(ti,1:2),X_R) && RobotsB.(qk)(ti) == 0 && RobotsB.(tauk)(ti) <= 0 ...
                 %     && pointInCircle(RobotsR.(z)(ti,1:2), RobotsB.(zk)(ti,1:2), gc) && RobotsR.mu(ti) == 0) ||...
             if   (pointInCircle(RobotsR.(z)(ti,1:2),FB,gf) && RobotsR.(q)(ti) == 0 && RobotsR.(eta)(ti) == 0 && RobotsB.mu(ti) == 1) ... % Red flag captured 
               || (pointInCircle(RobotsR.(z)(ti,1:2),FR,gf) && RobotsR.(q)(ti) == 0 && RobotsR.(eta)(ti) == 1 && RobotsB.mu(ti) == 0) % Team R Scored

                RobotsB.mu(ti) = 1 - RobotsB.mu(ti); % Jump in blue flag position state (captured or dropped) 
                RobotsR.(eta)(ti) = 1 - RobotsR.(eta)(ti); % Jump in carrying the flag state
            end
            %%%%%%%% First case accounted above: tagged in red zone w flag
            %if (pointInRectangle(RobotsR.(z)(ti,1:2),X_R) && RobotsR.(q)(ti) == 0 && RobotsR.(tau)(ti) <= 0 ...
             %       && pointInRectangle(RobotsB.(zk)(ti,1:2),X_R) && RobotsB.(qk)(ti) == 0 && RobotsB.(etak)(ti) == 1 ...
              %        && pointInCircle(RobotsB.(zk)(ti,1:2), RobotsR.(z)(ti,1:2), gc) && RobotsR.mu(ti) == 0) ||...
             if (pointInCircle(RobotsB.(zk)(ti,1:2),FR,gf) && RobotsB.(qk)(ti) == 0 && RobotsB.(etak)(ti) == 0 && RobotsR.mu(ti) == 1) ... % Red flag captured 
               || (pointInCircle(RobotsB.(zk)(ti,1:2),FB,gf) && RobotsB.(qk)(ti) == 0 && RobotsB.(etak)(ti) == 1 && RobotsR.mu(ti) == 0) ... % Team B Scored
               || (not(pointInRectangle(RobotsB.(zk)(ti,1:2),X)) && RobotsB.(qk)(ti) == 0 && RobotsB.(etak)(ti) == 1 && RobotsR.mu(ti) == 0) % Leaving X
                
                 if not(pointInRectangle(RobotsB.(zk)(ti,1:2),X))
                     RobotsB.(qk)(ti) = 1-RobotsB.(qk)(ti);
                 end
                RobotsB.(etak)(ti) = 1 - RobotsB.(etak)(ti); % Jump in carrying the flag state
                RobotsR.mu(ti) = 1 - RobotsR.mu(ti); % Jump in red flag position state (captured or dropped) 

            end
        end
        % Controller
        thetar = Angle_Control(ti,RobotsR,RobotsB,z,tau,q,eta,FR,FB,r,b, dt);
        % Flow of robot:
        RobotsR.(z)(ti+1,1:3) = Robot_Dynamics(RobotsR.(z)(ti,1:3),[50,thetar],dt);  
        % Flow of logic variables:
        RobotsR.(tau)(ti+1) = RobotsR.(tau)(ti) - dt;
        RobotsR.(q)(ti+1) = RobotsR.(q)(ti);
        RobotsR.(eta)(ti+1) = RobotsR.(eta)(ti);
    end
    RobotsR.mu(ti+1) = RobotsR.mu(ti); 
end

%%
% --------------------------------------------------------------
%%% Plot
% --------------------------------------------------------------
%
h=figure(1);
clf
set(0,'defaultfigurecolor',[1 1 1])
set(0,'defaulttextinterpreter','latex')
set(gcf,'color','w');


plot(x_f,y_f,'k','linewidth',2) % Boundary of PLaying Field
hold on 
plot([mean([x_f(1),x_f(2)]),mean([x_f(1),x_f(2)])],[y_f(end),y_f(end-1)],'k--','linewidth',2) % Half plane line

plot(FR(1)+gf*flx,FR(2)+gf*fly,'r--','linewidth',2) % Flag Region Team R
plot(FB(1)+gf*flx,FB(2)+gf*fly,'b--','linewidth',2) % Flag Region Team B

RedFlag = plot(FR(1),FR(2),'rx','linewidth',3,'markersize',8); % Flag Base
BlueFlag = plot(FB(1),FB(2),'bx','linewidth',3,'markersize',8); % Flag Base

grid on 
box on
axis equal
%hold off

xlim([-85 85]);
ylim([-45 45]);
set(gca,'TickLabelInterpreter','latex','fontsize', 11)
%
hz1B = animatedline('MaximumNumPoints',1,'Color','blue','Marker','.','markersize',20);
hz2B = animatedline('MaximumNumPoints',1,'Color','blue','Marker','.','markersize',20);
hz3B = animatedline('MaximumNumPoints',1,'Color','blue','Marker','.','markersize',20);
hztB = animatedline('MaximumNumPoints',1,'Color','green','Marker','.','markersize',20);
h22 = animatedline('MaximumNumPoints',2,'Color','blue','LineWidth',2);
hz1B2 = animatedline('MaximumNumPoints',2,'Color','blue','LineWidth',2);
hz2B2 = animatedline('MaximumNumPoints',2,'Color','blue','LineWidth',2);
hz3B2 = animatedline('MaximumNumPoints',2,'Color','blue','LineWidth',2);
hz1R = animatedline('MaximumNumPoints',1,'Color','red','Marker','.','markersize',20);
hz2R = animatedline('MaximumNumPoints',1,'Color','red','Marker','.','markersize',20);
hz3R = animatedline('MaximumNumPoints',1,'Color','red','Marker','.','markersize',20);
hz1R2 = animatedline('MaximumNumPoints',2,'Color','red','LineWidth',2);
hz2R2 = animatedline('MaximumNumPoints',2,'Color','red','LineWidth',2);
hz3R2 = animatedline('MaximumNumPoints',2,'Color','red','LineWidth',2);

[A,map] = rgb2ind(frame2im(getframe),256);
imwrite(A,map,'3.gif','LoopCount',65535,'DelayTime',0.01);
for ti=1:TSPAN(2)/(dt*5)
    %for k=1:b
        %Name =  sprintf('z_%dB',k);
        %sprintf('hz%dB = h2',k);
        %sprintf('addpoints(hz%dB,RobotsB.(Name)(ti,1),RobotsB.(Name)(ti,2))',k);
        %addpoints(h2,RobotsB.(Name)(ti,1),RobotsB.(Name)(ti,2));
        %addpoints(h22,[RobotsB.(Name)(ti,1) RobotsB.(Name)(ti,1)+6*cos(RobotsB.(Name)(ti,3))],[RobotsB.(Name)(ti,2) RobotsB.(Name)(ti,2)+6*sin(RobotsB.(Name)(ti,3))]);
        if RobotsB.tau_1B(ti) > 0
            hz1B.Color='#77AC30';
        else
            hz1B.Color='blue';
        end
        if sum(contains(fieldnames(RobotsB),'tau_2B')) && RobotsB.tau_2B(ti) > 0
            hz2B.Color='#77AC30';
        else
            hz2B.Color='blue';
        end
        if sum(contains(fieldnames(RobotsB),'tau_3B')) && RobotsB.tau_3B(ti) > 0
            hz3B.Color='#77AC30';
        else
            hz3B.Color='blue';
        end
        
        if RobotsB.q_1B(ti) == 1
            hz1B.Color='black';
        end
        if sum(contains(fieldnames(RobotsB),'q_2B')) && RobotsB.q_2B(ti) == 1
            hz2B.Color='black';
        end
        if sum(contains(fieldnames(RobotsB),'q_3B')) && RobotsB.q_3B(ti) == 1
            hz3B.Color='black';
        end

        if RobotsB.eta_1B(ti) == 1
            hz1B.Marker='pentagram';
        else
            hz1B.Marker='.';
        end
        if sum(contains(fieldnames(RobotsB),'eta_2B')) && RobotsB.eta_2B(ti) == 1
            hz2B.Marker='pentagram';
        else
            hz2B.Marker='.';
        end
        if sum(contains(fieldnames(RobotsB),'eta_3B')) && RobotsB.eta_3B(ti) == 1
            hz3B.Marker='pentagram';
        else
            hz3B.Marker='.';
        end

        if RobotsB.mu(ti) == 0
            BlueFlag.Color = 'white';
        else
            BlueFlag.Color = 'blue';
        end


        addpoints(hz1B,RobotsB.z_1B(ti,1),RobotsB.z_1B(ti,2));
        addpoints(hz1B2,[RobotsB.z_1B(ti,1) RobotsB.z_1B(ti,1)+6*cos(RobotsB.z_1B(ti,3))],[RobotsB.z_1B(ti,2) RobotsB.z_1B(ti,2)+6*sin(RobotsB.z_1B(ti,3))]);
        if sum(contains(fieldnames(RobotsB),'z_2B'))
            addpoints(hz2B,RobotsB.z_2B(ti,1),RobotsB.z_2B(ti,2));
            addpoints(hz2B2,[RobotsB.z_2B(ti,1) RobotsB.z_2B(ti,1)+6*cos(RobotsB.z_2B(ti,3))],[RobotsB.z_2B(ti,2) RobotsB.z_2B(ti,2)+6*sin(RobotsB.z_2B(ti,3))]);
        end
        if sum(contains(fieldnames(RobotsB),'z_3B'))
            addpoints(hz3B,RobotsB.z_3B(ti,1),RobotsB.z_3B(ti,2));
            addpoints(hz3B2,[RobotsB.z_3B(ti,1) RobotsB.z_3B(ti,1)+6*cos(RobotsB.z_3B(ti,3))],[RobotsB.z_3B(ti,2) RobotsB.z_3B(ti,2)+6*sin(RobotsB.z_3B(ti,3))]);
        end
        drawnow
        
        if RobotsR.tau_1R(ti) > 0
            hz1R.Color='#D95319';
        else
            hz1R.Color='red';
        end
        if sum(contains(fieldnames(RobotsR),'tau_2R')) && RobotsR.tau_2R(ti) > 0
            hz2R.Color='#D95319';
        else
            hz2R.Color='red';
        end
        if sum(contains(fieldnames(RobotsR),'tau_3R')) && RobotsR.tau_3R(ti) > 0
            hz3R.Color='#D95319';
        else
            hz3R.Color='red';
        end
        
        if RobotsR.q_1R(ti) == 1
            hz1R.Color='black';
        end
        if sum(contains(fieldnames(RobotsR),'q_2R')) &&  RobotsR.q_2R(ti) == 1
            hz2R.Color='black';
        end
        if sum(contains(fieldnames(RobotsR),'q_3R')) && RobotsR.q_3R(ti) == 1
            hz3R.Color='black';
        end

        if RobotsR.eta_1R(ti) == 1
            hz1R.Marker='pentagram';
        else
            hz1R.Marker='.';
        end
        if sum(contains(fieldnames(RobotsR),'eta_2R')) &&  RobotsR.eta_2R(ti) == 1
     
            hz2R.Marker='pentagram';
        else
            hz2R.Marker='.';
        end
        if sum(contains(fieldnames(RobotsR),'eta_3R')) && RobotsR.eta_3R(ti) == 1
            hz3R.Marker='pentagram';
        else
            hz3R.Marker='.';
        end

        if RobotsR.mu(ti) == 0
            RedFlag.Color = 'white';
        else
            RedFlag.Color = 'red';
        end
        
        addpoints(hz1R,RobotsR.z_1R(ti,1),RobotsR.z_1R(ti,2));
        addpoints(hz1R2,[RobotsR.z_1R(ti,1) RobotsR.z_1R(ti,1)+6*cos(RobotsR.z_1R(ti,3))],[RobotsR.z_1R(ti,2) RobotsR.z_1R(ti,2)+6*sin(RobotsR.z_1R(ti,3))]);
        if sum(contains(fieldnames(RobotsR),'z_2R'))
            addpoints(hz2R,RobotsR.z_2R(ti,1),RobotsR.z_2R(ti,2));
            addpoints(hz2R2,[RobotsR.z_2R(ti,1) RobotsR.z_2R(ti,1)+6*cos(RobotsR.z_2R(ti,3))],[RobotsR.z_2R(ti,2) RobotsR.z_2R(ti,2)+6*sin(RobotsR.z_2R(ti,3))]);
        end
        if sum(contains(fieldnames(RobotsR),'z_3R'))
            addpoints(hz3R,RobotsR.z_3R(ti,1),RobotsR.z_3R(ti,2));
            addpoints(hz3R2,[RobotsR.z_3R(ti,1) RobotsR.z_3R(ti,1)+6*cos(RobotsR.z_3R(ti,3))],[RobotsR.z_3R(ti,2) RobotsR.z_3R(ti,2)+6*sin(RobotsR.z_3R(ti,3))]);
        end
        drawnow
    if(mod(ti,5)==0)
        [A,map] = rgb2ind(frame2im(getframe),256);
        imwrite(A,map,'3.gif','WriteMode','append','DelayTime',0.01);
    end
end

saveas(h,'aq_setting','epsc')



