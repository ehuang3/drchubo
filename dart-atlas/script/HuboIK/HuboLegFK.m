function B = HuboLegFK(q,side)

%% Parameters
l1 = (79.5+107)/1000;
l2 = 88.43/1000;
l3 = (289.47-107)/1000;
l4 = 300.03/1000;
l5 = 300.38/1000;
l6 = 94.97/1000;

t = [    0, -pi/2,    0,    0,    0,    0];
f = [ pi/2, -pi/2,    0,    0, pi/2,    0];
r = [    0,     0,   l4,   l5,    0,   l6];
d = [    0,     0     0,    0,    0,    0];

n = numel(t);

T = @(t,f,r,d) [...
    cos(t), -sin(t)*cos(f),  sin(t)*sin(f), r*cos(t);...
    sin(t),  cos(t)*cos(f), -cos(t)*sin(f), r*sin(t);...
         0,         sin(f),         cos(f),        d;...
         0,              0,              0,        1];


neck = [...
        1   0   0   0
        0   1   0   0
        0   0   1 -l1
        0   0   0   1];
     
if strcmp(side,'right')
    waist = [...
        0  -1   0   0
        1   0   0 -l2
        0   0   1 -l3
        0   0   0   1];
    
    limits = [ -1.8,   -.6,  -1.3,    0,  -1.3,   -.2;...
                      0,     0,   1.4,  2.5,   1.8,    .3];
    
elseif strcmp(side,'left')
    waist = [...
        0  -1   0   0
        1   0   0  l2
        0   0   1 -l3
        0   0   0   1];
    
    limits = [    0,     0,  -1.3,     0,  -1.3,   -.3;...
                1.8,    .6,   1.4,   2.5,   1.8,    .2];
else
    error('HuboLegFK:side',...
    'Input argument "side" must either ''right'' or ''left''.')
end
    

%% Calculate
% B = eye(4);
B = waist*neck;
for i = 1:n
    i;
    B = B*T(t(i)+q(i),f(i),r(i),d(i));
end
end
