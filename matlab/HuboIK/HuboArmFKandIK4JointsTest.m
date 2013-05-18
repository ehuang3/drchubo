%% HuboTestFKandIK.m
% The "HuboTestFKandIK" script is used to test the forward and inverse
% kinematic equations for Hubo.
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
%   Created 16-DEC-2012
%-------------------------------------------------------------------------------

%% Clear
ccc
zeroSize = .0001;
l1 = 214.5/1000;
l2 = 179.14/1000;
l3 = 181.59/1000;

%% Arm test
arm = 'right';

type = 'r';

% px = sqrt(l2^2+l3^2);
% pIn = [(l2+l3)/2;0;0];
% 
% pIn
% BIn = [[eye(3) pIn];[0 0 0 1]]
% qOut = HuboArmIK4Joints(BIn,zeros(6,1),arm)

%%

qIn = [atan2(-l2,l3) 0 0 -pi/2]'
% qIn = 2*pi*rand(1,6)'-pi;

BIn = HuboArmFK4Joints(qIn,arm)
pIn = BIn(1:3,4)


qOut = HuboArmIK4Joints(BIn,zeros(6,1),arm)

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
