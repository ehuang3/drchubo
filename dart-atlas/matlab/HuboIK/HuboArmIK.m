function q = HuboArmIK(B,qPrev,arm)
% The "HuboArmIK" function computes the inverse kinematics for Hubo's arm.
%
% SYNTAX: TODO: Add syntax
%   q = HuboArmIK(B)
%   q = HuboArmIK(B,qPrev)
%   q = HuboArmIK(B,qPrev,arm)
%   q = HuboArmIK(B,qPrev,'PropertyName',PropertyValue,...)
% 
% INPUTS:
%   B - (4 x 4 SE(3) matrix) 
%       Homogenuous transformation matrix from hand to neck.
%
%   qPrev - (6 x 1 joint angles) [zeros(6,1)] 
%       Previous joint angles.
%
%   arm - ('right' or 'left') ['right']
%       Arm selection.
%
% PROPERTIES: TODO: Add properties
%   'propertiesName' - (size type) [defaultPropertyValue]
%       Description.
% 
% OUTPUTS:
%   q - (6 x 8 njoint angles) 
%       All possibel joint angle solutions.
%
% EXAMPLES: TODO: Add examples
%
% NOTES:
%
% NECESSARY FILES: TODO: Add necessary files
%   +somePackage, someFile.m
%
% SEE ALSO: TODO: Add see alsos
%    relatedFunction1 | relatedFunction2
%
% AUTHOR:
%    Rowland O'Flaherty (www.rowlandoflaherty.com)
%
% VERSION: 
%   Created 04-JAN-2013
%-------------------------------------------------------------------------------

%% Check Inputs

% Check number of inputs
narginchk(1,3)

% Apply default values
if nargin < 2, qPrev = zeros(6,1); end
if nargin < 3, arm = 'right'; end
 
% % Check input arguments for errors TODO: Add error checks
% assert(isnumeric(input1) && isreal(input1) && isequal(size(input1),[1,1]),...
%     'HuboArmIK:input1',...
%     'Input argument "input1" must be a ? x ? matrix of real numbers.')
% 
% % Get and check properties
% propargin = size(varargin,2);
% 
% assert(mod(propargin,2) == 0,...
%     'HuboArmIK:properties',...
%     'Properties must come in pairs of a "PropertyName" and a "PropertyValue".')
% 
% propStrs = varargin(1:2:propargin);
% propValues = varargin(2:2:propargin);
% 
% for iParam = 1:propargin/2
%     switch lower(propStrs{iParam})
%         case lower('propertyName')
%             propertyName = propValues{iParam};
%         otherwise
%             error('HuboArmIK:options',...
%               'Option string ''%s'' is not recognized.',propStrs{iParam})
%     end
% end
% 
% % Set to default value if necessary TODO: Add property defaults
% if ~exist('propertyName','var'), propertyName = defaultPropertyValue; end
% 
% % Check property values for errors TODO: Add property error checks
% assert(isnumeric(propertyName) && isreal(propertyName) && isequal(size(propertyName),[1,1]),...
%     'HuboArmIK:propertyName',...
%     'Property "propertyName" must be a ? x ? matrix of real numbers.')

%% Parameters
l1 = 214.5/1000;
l2 = 179.14/1000;
l3 = 181.59/1000;
l4 = 4.75*25.4/1000;

if strcmpi(arm,'right')
    neck = [...
        1   0   0   0
        0   0   1 -l1
        0  -1   0   0
        0   0   0   1];
    
    limits = [   -2,    -2,    -2,   -2,    -2,  -1.4;...
                  2,    .2,     2,    0,     2,   1.2];

elseif strcmpi(arm,'left')
    neck = [...
        1   0   0   0
        0   0   1  l1
        0  -1   0   0
        0   0   0   1];
    
    limits = [   -2,   -.2,    -2,    -2,    -2,  -1.4;...
                  2,     2,     2,     0,     2,   1.2];
else
    error('HuboArmIK:arm',...
    'Input argument "arm" must either ''right'' or ''left''.')
end

neckInv = neck^-1;
     
hand = [...
    1   0   0   0
    0   0  -1   0
    0   1   0   0
    0   0   0   1];

handInv = hand^-1;

zeroSize = .000001;

%% Variables
B = neckInv*B*handInv;
Binv = B^-1;
n = Binv(:,1);
nx = n(1); ny = n(2); nz = n(3);
s = Binv(:,2);
sx = s(1); sy = s(2); sz = s(3);
%%
a = Binv(:,3);
ax = a(1); ay = a(2); az = a(3);
p = Binv(:,4);
px = p(1); py = p(2); pz = p(3);

q1 = qPrev(1);
q3 = qPrev(3);

WSFlags = true(1,8);

% Initialize
q = nan(6,8);
m = [...
    1  1  1;...
    1  1 -1;...
    1 -1  1;...
    1 -1 -1;...
   -1  1  1;...
   -1  1 -1;...
   -1 -1  1;...
   -1 -1 -1];
    
%% Caculate
for i = 1:8
    % Solve for q4
    C4 = max(min((2*l4*px - l2^2 - l3^2 + l4^2 + px^2 + py^2 + pz^2)/(2*l2*l3),1),-1);
    
    if abs(C4 - 1) < zeroSize % Case 1: q4 == 0
        
        % Set q4
        q(4,i) = 0;
        
        % Set q3
        q(3,i) = q3;

        % Solve for q6
        S6 = py/(l2 + l3);
        C6 = -(l4 + px)/(l2 + l3);
        q(6,i) = atan2(S6,C6);
        
        % Solve for q2
        S2 = C4*C6*ax - C4*S6*ay;
        if abs(S2 - 1) < zeroSize
            q(2,i) = pi/2;
        elseif abs(S2 + 1) < zeroSize
            q(2,i) = -pi/2;
        else
            WSFlags(i) = WSFlags(i) & (1-S2^2 >= 0);
            q(2,i) = atan2(S2,m(i,3)*real(sqrt(1-S2^2)));
        end
        
        % Solve for q5
        qT = atan2(- C6*ay - S6*ax,az);
        C2 = cos(q(2,i));
        
        if abs(C2) < zeroSize % Case 3: q2 = pi/2 or -pi/2
            q(1,i) = q1;
            q(3,i) = q3;
            
            % Solve for q5
            if S2 > 0 % Case 3a: q2 = pi/2
                qT = atan2(nz,-sz);
                q(5,i) = wrapToPi(q(1,i) - q(3,i) - qT);
            else % Case 3b: q2 = -pi/2
                qT = atan2(-nz,sz);
                q(5,i) = wrapToPi(qT - q(1,i) - q(3,i));
            end
            
        else
            if C2 < 0
                qT = qT + pi;
            end
            q(5,i) = wrapToPi(qT - q(3,i));
            
            % Solve for q1
            q(1,i) = atan2(S6*ny - C6*nx,C6*sx - S6*sy);
            if C2 < 0
                q(1,i) = q(1,i) + pi;
            end
            q(1,i) = wrapToPi(q(1,i));
        end

    else

        % Sove for q4
        WSFlags(i) = WSFlags(i) & (1-C4^2 >= 0);
        q(4,i) = atan2(m(i,1)*real(sqrt(1-C4^2)),C4);
        
        % Solve for q5
        S4 = sin(q(4,i));
        S5 = pz/(S4*l2);
        if abs(S5 - 1) < zeroSize
            q(5,i) = pi/2;
        elseif abs(S5 + 1) < zeroSize
            q(5,i) = -pi/2;
        else
            WSFlags(i) = WSFlags(i) & (1-S5^2 >= 0);
            q(5,i) = atan2(S5,m(i,2)*real(sqrt(1-S5^2)));
        end
        
        % Solve for q6
        C5 = cos(q(5,i));
        S6 = (C5*S4*l2 + (py*(l3 + C4*l2 - (C5*S4*l2*py)/(l4 + px)))/(l4 + px + py^2/(l4 + px)))/(l4 + px);
        C6 = -(l3 + C4*l2 - (C5*S4*l2*py)/(l4 + px))/(l4 + px + py^2/(l4 + px));
        q(6,i) = atan2(S6,C6);
        
        % S6psi = S4*C5*l2;
        % C6psi = -C4*l2-l3;
        % psi = atan2(py,px+l4);
        % q(6,i) = wrapToPi(atan2(S6psi,C6psi)-psi)
        
        % Solve for q2
        S2 = ax*(C4*C6 - C5*S4*S6) - ay*(C4*S6 + C5*C6*S4) - S4*S5*az;
        if abs(S2 - 1) < zeroSize
            q(2,i) = pi/2;
        elseif abs(S2 + 1) < zeroSize
            q(2,i) = -pi/2;
        else
            WSFlags(i) = WSFlags(i) & (1-S2^2 >= 0);
            q(2,i) = atan2(S2,m(i,3)*real(sqrt(1-S2^2)));
        end
        
        % Solve for q3
        C2 = cos(q(2,i));
        
        if abs(C2) < zeroSize % Case 2: q2 = pi/2 or -pi/2
            q(3,i) = q3;
            
            % Solve for q1
            if S2 > 0 % Case 2a: q2 = pi/2
                qT = atan2(S6*sy - C6*sx,S6*ny - C6*nx);
                if S4 < 0,
                    qT = qT + pi;
                end
                q(1,i) = wrapToPi(qT + q(3,i));
            else % Case 2b: q2 = -pi/2
                qT = atan2(S6*sy - C6*sx,S6*ny - C6*nx);
                if S4 < 0
                    qT = qT + pi;
                end
                q(1,i) = wrapToPi(qT - q(3,i));
            end
            
            
        else
            q(3,i) = atan2(S4*S6*ay - C4*S5*az - C6*S4*ax - C4*C5*C6*ay - C4*C5*S6*ax,C5*az - C6*S5*ay - S5*S6*ax);
            if C2 < 0
                q(3,i) = q(3,i) - pi;
            end
            q(3,i) = wrapToPi(q(3,i));
            
            % Solve for q1
            q(1,i) = atan2(C4*S6*ny - C4*C6*nx + S4*S5*nz + C5*C6*S4*ny + C5*S4*S6*nx,C4*C6*sx - C4*S6*sy - S4*S5*sz - C5*C6*S4*sy - C5*S4*S6*sx);
            if C2 < 0
                q(1,i) = q(1,i) + pi;
            end
            q(1,i) = wrapToPi(q(1,i));
            
        end
    end
    
end

if strcmpi(arm,'right')
    q(2,:) = wrapToPi(q(2,:) + limits(2,2));
elseif strcmpi(arm,'left')
    q(2,:) = wrapToPi(q(2,:) + limits(1,2));
end

qInLimits = all((q > repmat(limits(1,:)'-zeroSize,1,8)) & (q < repmat(limits(2,:)'+zeroSize,1,8)),1);
if any(qInLimits & WSFlags)
    q = q(:,qInLimits);
    [~,I] = min(sum(q.^2,1));
    q = q(:,I);
else
    % Apply limits
    q = min(max(q,repmat(limits(1,:)',1,8)),repmat(limits(2,:)',1,8));
    pTilde = nan(3,8);
    for i = 1:8
        BTilde = HuboArmFK(q(:,i),arm);
        BTilde = neckInv*BTilde*handInv;
        pTilde(:,i) = BTilde(1:3,4);
    end
    pDist = sum((repmat(p(1:3),1,8) - pTilde).^2,1);
    [~,I] = min(pDist);
    if pDist(I) > .1
        q = qPrev;
    else
        q = q(:,I);
    end
end

end