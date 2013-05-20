function q = HuboArmIK(B,qPrev,side)

%% Parameters
l1 = (79.5+107)/1000;
l2 = 88.43/1000;
l3 = (289.47-107)/1000;
l4 = 300.03/1000;
l5 = 300.38/1000;
l6 = 94.97/1000;

neck = [...
        1   0   0   0
        0   1   0   0
        0   0   1 -l1
        0   0   0   1];

if strcmpi(side,'right')
    waist = [...
        0  -1   0   0
        1   0   0 -l2
        0   0   1 -l3
        0   0   0   1];
    
    limits = [ -1.8,   -.6,  -1.3,    0,  -1.3,   -.2;...
                  0,     0,   1.4,  2.5,   1.8,    .3];

elseif strcmpi(side,'left')
    waist = [...
        0  -1   0   0
        1   0   0  l2
        0   0   1 -l3
        0   0   0   1];
    
    limits = [    0,     0,  -1.3,     0,  -1.3,   -.3;...
                1.8,    .6,   1.4,   2.5,   1.8,    .2];
else
    error('HuboLegIK:side',...
    'Input argument "side" must either ''right'' or ''left''.')
end

neckInv = neck^-1;

waistInv = waist^-1;

zeroSize = .000001;

%% Variables
B = neckInv*waistInv*B;

Binv = B^-1;
n = Binv(:,1);
nx = n(1); ny = n(2); nz = n(3);
s = Binv(:,2);
sx = s(1); sy = s(2); sz = s(3);
a = Binv(:,3);
ax = a(1); ay = a(2); az = a(3);
p = Binv(:,4);
px = p(1); py = p(2); pz = p(3);

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
    C4 = ((l6 + px)^2 - l4^2 - l5^2 + py^2 + pz^2)/(2*l4*l5);
    WSFlags(i) = WSFlags(i) & (1-C4^2 >= -zeroSize);
    q(4,i) = atan2(m(i,1)*real(sqrt(1-C4^2)),C4);
    
    S4 = sin(q(4,i));
    psi = atan2(S4*l4, C4*l4+l5);
    q(5,i) = wrapToPi(atan2(-pz, m(i,2)*real(sqrt((px+l6)^2+py^2)))-psi);
    
    
    q(6,i) = atan2(py, -px-l6);
    C45 = cos(q(4,i)+q(5,i));
    C5 = cos(q(5,i));
    if C45*l4 + C5*l5 < 0
        q(6,i) = wrapToPi(q(6,i) + pi);
    end
    
    S6 = sin(q(6,i));
    C6 = cos(q(6,i));
    
    S2 = C6*ay + S6*ax;
    
    WSFlags(i) = WSFlags(i) & (1-S2^2 >= -zeroSize);
    q(2,i) = atan2(S2,m(i,3)*real(sqrt(1-S2^2)));
    C2 = cos(q(2,i));
    
    q(1,i) = atan2(C6*sy + S6*sx,C6*ny + S6*nx);
    if C2 < 0
        q(1,i) = wrapToPi(q(1,i) + pi);
    end
    
    
    C2 = cos(q(2,i));
    q345 = atan2(-az/C2,-(C6*ax - S6*ay)/C2);
    q(3,i) = wrapToPi(q345-q(4,i)-q(5,i));
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
        BTilde = HuboArmFK(q(:,i),side);
        BTilde = neckInv*waistInv*BTilde;
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
