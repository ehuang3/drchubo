
%% Clear
ccc
zeroSize = .0001;

l1 = 214.5/1000;
l2 = 179.14/1000;
l3 = 181.59/1000;
l4 = 4.75*25.4/1000;
ld = 7*25.4/1000;
ad = pi/4;

side = 'right';

type = 's';

script = 'w';

%% Teleop

if script == 't'
    tx = 0;
    ty = 0;
    tz = 0;
    
    pyLimits = [-0.2050 0.1850];
    
    px = 0.0925;
    py = min(max(ty,pyLimits(1)),pyLimits(2));
    pz = 0.04;
    
    px0 = (l3+l4)*sin(pi/4)+ld;
    py0 = -l1;
    pz0 = (l3+l4)*cos(pi/4)-l2;
    
    pIn = [px0+px;py0+py;pz0+pz]
   
    BIn = [[eye(3) pIn];[0 0 0 1]]
    
    qOut = HuboDrillIK(BIn,zeros(6,1),side)
    
    BOut = HuboDrillFK(qOut,side)
    pOut = BOut(1:3,4)
    
    all(abs(pOut - repmat(pIn,1,size(pOut,2))) < zeroSize)
end

%% Give point on wall
if script == 'p'

%     px = 0;
%     py = 0;
%     pz = 0;
    
    % Extreme workspace points min
%     px = 0.0925;
%     py = -0.2050;
%     pz = 0.04;

    % Extreme workspace points max
    px = 0.0925;
    py = 0.1850;
    pz = 0.04;
    
    px0 = (l3+l4)*sin(pi/4)+ld;
    py0 = -l1;
    pz0 = (l3+l4)*cos(pi/4)-l2;
    
    pIn = [px0+px;py0+py;pz0+pz]
   
    BIn = [[eye(3) pIn];[0 0 0 1]]
    
    qOut = HuboDrillIK(BIn,zeros(6,1),side)
    
    BOut = HuboDrillFK(qOut,side)
    pOut = BOut(1:3,4)
    
    all(abs(pOut - repmat(pIn,1,size(pOut,2))) < zeroSize)
    
end

%% Give joint angles
if script == 'j' 
    qIn = 2*pi*rand(1,6)'-pi;
    
    switch type
        case '0'
            fprintf(1,'Case 0: All zeros\n');
            qIn = zeros(6,1);
            
        case 's'
            fprintf(1,'Case Special\n');
            qIn = [
                -pi/2
                0
                0
                -pi/4
                -pi/2
                0];
            
        otherwise
            fprintf(1,'Case Random\n');
    end
    qIn
    
    BIn = HuboDrillFK(qIn,side)
    
    qOut = HuboDrillIK(BIn,qIn,side)
    
    abs(qOut - repmat(qIn,1,size(qOut,2))) < zeroSize
    
    all(abs(qOut - repmat(qIn,1,size(qOut,2))) < zeroSize,1)
    
    
    if any(sum(abs(repmat(qIn,1,size(qOut,2)) - qOut)) < zeroSize)
        [~,I] = min(sum(abs(repmat(qIn,1,size(qOut,2)) - qOut)));
    else
        I = nan;
    end
    % end
    I
    if ~isnan(I)
        qOut(:,I)
    end
end

%% Test workspace
if script == 'w'

    % Paper figure
    PX = .025:.025:.15;
    PY = -.225:.015:.225;
    PZ = -.015:.015:.105;
    
    AX = 0;
    AY = 0;
    AZ = 0;
    
    % Higher res
%     PX = .08:.0025:.095;
%     PY = -.225:.015:.225;
%     PZ = .015:.005:.075;
    
    % Best workspace values
%     PX = .0925:.0025:.0925;
%     PY = -.220:.015:.205;
%     PZ = .0375:.0025:.0425;

    % Test values
%     PX = .05:.05:.3;
%     PY = -.450:.06:.450;
%     PZ = -.03:.06:.21;
    
%     AX = -pi/4:pi/10:pi/4;
%     AY = -pi/4:pi/10:pi/4;
%     AZ = -pi/4:pi/10:pi/4;
    
    V = nan(numel(PX),numel(PY),numel(PZ));
    
    for ix = 1:numel(PX)
        px = PX(ix);
        fprintf(1,'Wall distance: %.3f\n',px);
        for iy = 1:numel(PY)
             py = PY(iy);
            for iz = 1:numel(PZ)
                pz = PZ(iz);
                
                px0 = (l3+l4)*sin(pi/4)+ld;
                py0 = -l1;
                pz0 = (l3+l4)*cos(pi/4)-l2;
                
                pIn = [px0+px;py0+py;pz0+pz];
                
                validPos = false;
                for iax = 1:numel(AX)
                    for iay = 1:numel(AY)
                        for iaz = 1:numel(AZ)
                            RIn = rotRep('x',AX(iax))*rotRep('y',AY(iay))*rotRep('z',AZ(iaz));
                            
                            BIn = [[RIn pIn];[0 0 0 1]];
                            
                            qOut = HuboDrillIK(BIn,zeros(6,1),side);
                            
                            BOut = HuboDrillFK(qOut,side);
                            pOut = BOut(1:3,4);
                            
                            validPos = all(abs(pOut - repmat(pIn,1,size(pOut,2))) < zeroSize);
                            if validPos
                                break;
                            end
                        end
                        if validPos
                            break;
                        end
                    end
                    if validPos
                        break;
                    end
                end
                V(iy,iz,ix) = validPos;
                if V(iy,iz,ix)
                    scatter3(px,py,pz,50,'g','filled')
%                     scatter3(px+px0,py+py0,pz+pz0,50,'g','filled')
                    hold on
                else
                    scatter3(px,py,pz,50,'r','filled')
%                     scatter3(px+px0,py+py0,pz+pz0,50,'r','filled')
                    hold on
                end
                

    
            end
        end
        if ix == 1
            set(1,'Position',[1   346   840   633])
            view(167,44)
            title('Workspace Mapping')
            xlabel('X [m]');
            ylabel('Y [m]');
            zlabel('Z [m]');
            
            xlim([PX(1) PX(end)])
            ylim([PY(1) PY(end)]+[-1 1]*.05)
            zlim([PZ(1) PZ(end)]+[0 1]*.05)
            
%             xlim([PX(1) PX(end)]+px0)
%             ylim([PY(1) PY(end)]+py0+[-1 1]*.05)
%             zlim([PZ(1) PZ(end)]+pz0+[0 1]*.05)
        end
        drawnow
    end
    hold off
    figBoldify(1,'fontSize',20)
end


%% Saving figure
if 1
    set(1,'PaperPositionMode','auto')
    saveas(1,'eeWorkspace.png')
    saveas(1,'eeWorkspace.fig')
end
