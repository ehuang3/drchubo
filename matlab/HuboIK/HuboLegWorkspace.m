%% HuboLegWorkspace.m
% The "HuboLegWorkspace" script ...  TODO: Add description
%
% NOTES:
%
% NECESSARY FILES AND/OR PACKAGES: TODO: Add necessary files
%   +somePackage, someFile.m
%
% SEE ALSO: TODO: Add see alsos
%    relatedFunction1 | relatedFunction2
%
% AUTHOR:
%    Rowland O'Flaherty (www.rowlandoflaherty.com)
%
% VERSION: 
%   Created 21-FEB-2013
%-------------------------------------------------------------------------------

%% Clear
ccc
zeroSize = .0001;

%% Parameters
loadFlag = true;
saveFlag = false;

l1 = (79.5+107)/1000;
l2 = 88.43/1000;
l3 = (289.47-107)/1000;
l4 = 300.03/1000;
l5 = 300.38/1000;
l6 = 94.97/1000;

side = 'right';

PX = -.6:.025:.6;
PY = -.175:.0125:0;
PZ = 0:.0125:.4125;

%% Get default position
B0 = HuboLegFK(zeros(6,1),side);

nX = numel(PX);
nY = numel(PY);
nZ = numel(PZ);

validPos = false(nX,nY,nZ);

%% Set figure
figure(1)
set(1,'Position',[1   517   560   462])

%% Run
if ~loadFlag
BIn = B0;
for ix = 1:nX
    for iy = 1:nY
        for iz = 1:nZ
            BIn(1:3,4) = B0(1:3,4) + [PX(ix);PY(iy);PZ(iz)];
            
            q = HuboLegIK(BIn,zeros(6,1),side);
            BOut = HuboLegFK(q,side);
            
            if sum(abs(BOut(:) - BIn(:))) < .01
                validPos(ix,iy,iz) = true;
                scatter3(PX(ix),PY(iy),PZ(iz),35,'g','filled')
                hold on
%             else
%                 scatter3(PX(ix),PY(iy),PZ(iz),25,'r','filled')
%                 hold on
            end
        end
    end
    drawnow
end
else
    load('LegWorkspaceFigs/legWorkspace.mat')
    open('LegWorkspaceFigs/legWorkspace.fig')
end
hold off
xlabel('x')
ylabel('y')
zlabel('z')
axis equal
xlim([PX(1) PX(end)])
ylim([PY(1) PY(end)])
zlim([PZ(1) PZ(end)])
view([-253 14])
    

%% Save
if saveFlag
    saveas(1,'LegWorkspaceFigs/legWorkspace.fig')
    view([1 0 0])
    saveas(1,'LegWorkspaceFigs/legWorkspaceXView.png')
    view([0 1 0])
    saveas(1,'LegWorkspaceFigs/legWorkspaceYView.png')
    view([0 0 1])
    saveas(1,'LegWorkspaceFigs/legWorkspaceZView.png')
    
    save('LegWorkspaceFigs/legWorkspace.mat')
end


%%

