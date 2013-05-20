function q = HuboArmIK4Joints(B,qPrev,arm)

%% Parameters
l1 = 214.5/1000;
l2 = 179.14/1000;
l3 = 181.59/1000;

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
    
    limits = [   -2,   -.3,    -2,    -2,    -2,  -1.4;...
                  2,     2,     2,     0,     2,   1.2];
else
    error('HuboArmIK:arm',...
    'Input argument "arm" must either ''right'' or ''left''.')
end

neckInv = neck^-1;
     
wrist = [...
        1   0   0   0
        0   1   0   0
        0   0   1 -l3
        0   0   0   1];

wristInv = wrist^-1;

zeroSize = .000001;

%% Variables
% B = neckInv*B*wristInv;
% B = neckInv*B;
% n = B(:,1);
% nx = n(1); ny = n(2); nz = n(3);
% s = B(:,2);
% sx = s(1); sy = s(2); sz = s(3);
% a = B(:,3);
% ax = a(1); ay = a(2); az = a(3);
p = B(:,4);
px = p(1); py = p(2); pz = p(3);

Binv = B^-1;
% in = Binv(:,1);
% inx = in(1); iny = in(2); inz = in(3);
% is = Binv(:,2);
% isx = is(1); isy = is(2); isz = is(3);
% ia = Binv(:,3);
% iax = ia(1); iay = ia(2); iaz = ia(3);
ip = Binv(:,4);
ipx = ip(1); ipy = ip(2); ipz = ip(3);

q1 = qPrev(1);
q3 = qPrev(3);

% Initialize
q = nan(4,4);
m = [....
    1  1;...
    1 -1;...
   -1  1;...
   -1 -1];
    
%% Caculate
for i = 1:4
    % Solve for q4
%     C4 = (px^2 + py^2 + pz^2 - l2^2 - l3^2) / (2*l2*l3);
%     q(4,i) = atan2(m(i,1)*real(sqrt(1-C4^2)),C4);
%     S4 = sin(q(4,i));
%     
%     % Solve for q2
%     S2 = (pz + az*l3)/l2;
%     q(2,i) = atan2(S2,m(i,2)*real(sqrt(1-S2^2)));
%     C2 = cos(q(2,i));

    S4 = max(min(-ipx/l2,1),-1);
    q(4,i) = atan2(S4,m(i,1)*real(sqrt(1-S4^2)));
    C4 = cos(q(4,i));
    
    % Solve for q1
%     q(1,i) = atan2(-px,py);
    q(1,i) = q1;
    S1 = sin(q(1,i));
    C1 = cos(q(1,i));
    
%     % Solve for q2
%     if m(i,2) == 1
%         S2 = (l2*pz + C1*py*(pz^2 - C4^2*l3^2 - l2^2 + C1^2*py^2 + S1^2*px^2 - 2*C4*l2*l3 - px*py*sin(2*q1))^(1/2) - S1*px*(pz^2 - C4^2*l3^2 - l2^2 + C1^2*py^2 + S1^2*px^2 - 2*C4*l2*l3 - px*py*sin(2*q1))^(1/2) + C4*l3*pz)/(pz^2 + C1^2*py^2 + S1^2*px^2 - px*py*sin(2*q1));
%         C2 = -(pz*(pz^2 - C4^2*l3^2 - l2^2 + C1^2*py^2 + S1^2*px^2 - 2*C4*l2*l3 - 2*C1*S1*px*py)^(1/2) - C1*py*(l2 + C4*l3) + S1*px*(l2 + C4*l3))/(pz^2 + C1^2*py^2 + S1^2*px^2 - 2*C1*S1*px*py);
%     else
%         S2 = (l2*pz - C1*py*(pz^2 - C4^2*l3^2 - l2^2 + C1^2*py^2 + S1^2*px^2 - 2*C4*l2*l3 - px*py*sin(2*q1))^(1/2) + S1*px*(pz^2 - C4^2*l3^2 - l2^2 + C1^2*py^2 + S1^2*px^2 - 2*C4*l2*l3 - px*py*sin(2*q1))^(1/2) + C4*l3*pz)/(pz^2 + C1^2*py^2 + S1^2*px^2 - px*py*sin(2*q1));
%         C2 = (pz*(pz^2 - C4^2*l3^2 - l2^2 + C1^2*py^2 + S1^2*px^2 - 2*C4*l2*l3 - 2*C1*S1*px*py)^(1/2) + C1*py*(l2 + C4*l3) - S1*px*(l2 + C4*l3))/(pz^2 + C1^2*py^2 + S1^2*px^2 - 2*C1*S1*px*py);
%     end
%     S2 = real(S2);
%     C2 = real(C2);
%     q(2,i) = atan2(S2,C2);
%     S2 = sin(q(2,i));
%     C2 = cos(q(2,i));
%     
%     
%     % Solve for q3
%     if abs(C4 - 1) < zeroSize % Case 1: q4 == 0
%         q(3,i) = q3;
%     else
%         S3 = ((C2*pz - C1*S2*py + S1*S2*px)*(S4*l2 - S2*S4*pz - C1*C2*S4*py + C2*S1*S4*px))/(C4*(C1^2*px^2 + C2^2*pz^2 + S1^2*py^2 + C1^2*S2^2*py^2 + S1^2*S2^2*px^2 + 2*C1*S1*px*py - 2*C1*C2*S2*py*pz + 2*C2*S1*S2*px*pz - 2*C1*S1*S2^2*px*py));
%         C3 = (C1*S4*l2*px + S1*S4*l2*py - C1*S2*S4*px*pz - S1*S2*S4*py*pz + C1*C2*S1*S4*px^2 - C1*C2*S1*S4*py^2 - C1^2*C2*S4*px*py + C2*S1^2*S4*px*py)/(C4*(C1^2*px^2 + C2^2*pz^2 + S1^2*py^2 + C1^2*S2^2*py^2 + S1^2*S2^2*px^2 + 2*C1*S1*px*py - 2*C1*C2*S2*py*pz + 2*C2*S1*S2*px*pz - 2*C1*S1*S2^2*px*py));
%         q(3,i) = atan2(S3,C3);
%     end
    
end

% if strcmpi(arm,'right')
%     q(2,:) = wrapToPi(q(2,:) + limits(2,2));
% elseif strcmpi(arm,'left')
%     q(2,:) = wrapToPi(q(2,:) + limits(1,2));
% end

end
