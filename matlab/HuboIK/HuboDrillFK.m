function B = HuboDrillFK(q,arm)

assert(size(q,2) == 1);

%% Parameters
l1 = 214.5/1000;
l2 = 179.14/1000;
l3 = 181.59/1000;
l4 = 4.75*25.4/1000;
ld = 7*25.4/1000;
ad = pi/4;

t =      [ pi/2, -pi/2,  pi/2,     0,     0, pi/2];
f =      [ pi/2,  pi/2, -pi/2,  pi/2, -pi/2,    0];
r =      [    0,     0,     0,     0,     0,   l4];
d =      [    0,     0,   -l2,     0,   -l3,    0];

n = numel(t);

T = @(t,f,r,d) [...
    cos(t), -sin(t)*cos(f),  sin(t)*sin(f), r*cos(t);...
    sin(t),  cos(t)*cos(f), -cos(t)*sin(f), r*sin(t);...
         0,         sin(f),         cos(f),        d;...
         0,              0,              0,        1];


if strcmp(arm,'right')
    neck = [...
        1   0   0   0
        0   0   1 -l1
        0  -1   0   0
        0   0   0   1];
    
    limits = [   -2,    -2,    -2,   -2.5,    -2,  -1.4;...
                  2,    .3,     2,    0,     2,   1.2];
    
    q(2) = q(2) - limits(2,2);
    
elseif strcmp(arm,'left')
    neck = [...
        1   0   0   0
        0   0   1  l1
        0  -1   0   0
        0   0   0   1];
    
    limits = [   -2,   -.3,    -2,    -2.5,    -2,  -1.4;...
                  2,     2,     2,     0,     2,   1.2];
     
    q(2) = q(2) - limits(1,2);
  
else
    error('HuboArmIK:arm',...
    'Input argument "arm" must either ''right'' or ''left''.')
end
     
drill = [...
    cos(ad) 0   sin(ad)   ld*cos(ad)
    0       1   0         0
   -sin(ad) 0   cos(ad)   -ld*sin(ad)
    0       0   0         1];

%% Calculate
% B = eye(4);
B = neck;
for i = 1:n
    B = B*T(t(i)+q(i),f(i),r(i),d(i));
end
B = B*drill;

end
