%% HuboLegFKandIKTest.m
% The "HuboArmTestFKandIK" script is used to test the forward and inverse
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

l1 = (79.5+107)/1000;
l2 = 88.43/1000;
l3 = (289.47-107)/1000;
l4 = 300.03/1000;
l5 = 300.38/1000;
l6 = 94.97/1000;



%% Leg test
side = 'right';

type = 's';

cnt = 0;
maxCnt = 1;
while (cnt < maxCnt)
    cnt = cnt + 1;
    
    qIn = 2*pi*rand(1,6)'-pi;
    
    switch type
        case '0'
            fprintf(1,'Case 0: All zeros\n');
            qIn = zeros(6,1);
            
        case '1'
            fprintf(1,'Case 1: q4 = 0\n');
            qIn(4) = 0;
            
        case '2a'
            fprintf(1,'Case 2a: q2 = pi/2\n');
            qIn(2) = pi/2;
            
        case '2b'
            fprintf(1,'Case 2b: q2 = -pi/2\n');
            qIn(2) = -pi/2;
            
        case '3a'
            fprintf(1,'Case 3a: q4 = 0 && q2 = pi/2\n');
            qIn(2) = pi/2;
            qIn(4) = 0;
            
        case '3b'
            fprintf(1,'Case 3b: q4 = 0 && q2 = -pi/2\n');
            qIn(2) = -pi/2;
            qIn(4) = 0;
            
        case 's'
            fprintf(1,'Case Special\n');
            
            qIn = [
                -1.13097
-0.57964
-0.53242
0.0558357
-0.23662
0.18048];
            
        otherwise
            fprintf(1,'Case Random\n');
    end
    qIn
    
    nEqns = 8;
    BArm = HuboLegFK(qIn,side)
    
    qOut = HuboLegIK(BArm,qIn,side)
    
%     abs(qOut - repmat(qIn,1,nEqns)) < zeroSize
    
%     all(abs(qOut - repmat(qIn,1,nEqns)) < zeroSize,1)
    
    
%     if any(sum(abs(repmat(qIn,1,nEqns) - qOut)) < zeroSize)
%         [~,I] = min(sum(abs(repmat(qIn,1,nEqns) - qOut)));
%     else
%         I = nan;
%     end
%     % end
%     I
%     if ~isnan(I)
%         qOut(:,I)
%     end
%     
%     if isnan(I)
%         pause
%     end
end

% fprintf(1,'ALL GOOD!\n');
