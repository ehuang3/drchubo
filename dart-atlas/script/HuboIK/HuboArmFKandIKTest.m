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

%% Arm test
arm = 'right';

type = 's';

qArm = 2*pi*rand(1,6)'-pi;

switch type
    case '0'
        fprintf(1,'Case 0: All zeros\n');
        qArm = zeros(6,1);
        
    case '1'
        fprintf(1,'Case 1: q4 = 0\n');
        qArm(4) = 0;
        
    case '2a' 
        fprintf(1,'Case 2a: q2 = pi/2\n');
        qArm(2) = pi/2;
        
    case '2b'
        fprintf(1,'Case 2b: q2 = -pi/2\n');
        qArm(2) = -pi/2;
        
    case '3a'
        fprintf(1,'Case 3a: q4 = 0 && q2 = pi/2\n');
        qArm(2) = pi/2;
        qArm(4) = 0;
        
    case '3b'
        fprintf(1,'Case 3b: q4 = 0 && q2 = -pi/2\n');
        qArm(2) = -pi/2;
        qArm(4) = 0;
        
    case 's'
        fprintf(1,'Case Special\n');
        qArm = [
                0.3438
   -1.3708
         0
   -0.4908
   -1.5708
   -0.3438];
        
    otherwise
        fprintf(1,'Case Random\n');
end
qArm

BArm = HuboArmFK(qArm,arm);

qArmOut = HuboArmIK(BArm,qArm,arm)

abs(qArmOut - repmat(qArm,1,size(qArmOut,2))) < zeroSize

all(abs(qArmOut - repmat(qArm,1,size(qArmOut,2))) < zeroSize,1)


if any(sum(abs(repmat(qArm,1,size(qArmOut,2)) - qArmOut)) < zeroSize)
    [~,I] = min(sum(abs(repmat(qArm,1,size(qArmOut,2)) - qArmOut)));
else
    I = nan;
end
% end
I
if ~isnan(I)
    qArmOut(:,I)
end
