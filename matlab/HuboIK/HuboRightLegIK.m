function q = HuboRightLegIK(B)


%% Parameters
l1 = 214.5;
l2 = 179.14;
l3 = 181.59;
l4 = 10;
l5 = 10;

%% Variables
Binv = B^-1;
n = Binv(:,1);
nx = n(1); ny = n(2); nz = n(3);
s = Binv(:,2);
sx = s(1); sy = s(2); sz = s(3);
a = Binv(:,3);
ax = a(1); ay = a(2); az = a(3);
p = Binv(:,4);
px = p(1); py = p(2); pz = p(3);

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
    C4 = ((l5 + px)^2 - l3^2 - l4^2 + py^2 + pz^2)/(2*l3*l4);
    q(4,i) = atan2(m(i,1)*sqrt(1-C4^2),C4);
    
    S4 = sin(q(4,i));
    psi = atan2(S4*l3, C4*l3+l4);
    q(5,i) = atan2(-pz, m(i,2)*sqrt((px+l5)^2+py^2))-psi;

    q(6,i) = atan2(py, -px-l5);
    
    S6 = sin(q(6,i));
    C6 = cos(q(6,i));
    
    S2 = C6*ay + S6*ax;
    q(2,i) = atan2(S2,m(i,3)*sqrt(1-S2^2));
    
    q(1,i) = wrapToPi(atan2(C6*sy + S6*sx,C6*ny + S6*nx));
    
    C2 = cos(q(2,i));
    q345 = atan2(-az/C2,-(C6*ax - S6*ay)/C2);
    q(3,i) = q345-q(4,i)-q(5,i);
end

end
