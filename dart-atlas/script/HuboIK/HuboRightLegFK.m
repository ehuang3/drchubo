function B = HuboRightLegFK(q)


%% Parameters
l1 = 214.5;
l2 = 179.14;
l3 = 181.59;
l4 = 10;
l5 = 10;

t = [    0, -pi/2,    0,    0,    0,    0];
f = [ pi/2, -pi/2,    0,    0, pi/2,    0];
r = [    0,     0,   l3,   l4,    0,   l5];
d = [    0,     0     0,    0,    0,    0];

n = numel(t);

T = @(t,f,r,d) [...
    cos(t), -sin(t)*cos(f),  sin(t)*sin(f), r*cos(t);...
    sin(t),  cos(t)*cos(f), -cos(t)*sin(f), r*sin(t);...
         0,         sin(f),         cos(f),        d;...
         0,              0,              0,        1];

%% Calculate
B = eye(4);
for i = 1:n
    B = B*T(t(i)+q(i),f(i),r(i),d(i));
end

end
