function [ u ] = AtlasLegIK( Tf )
%ATLASLEGIK Joint angles given foot transform

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Vars
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
x0 = Tf(1,1); y0 = Tf(1,2); z0 = Tf(1,3); t0 = Tf(1,4);
x1 = Tf(2,1); y1 = Tf(2,2); z1 = Tf(2,3); t1 = Tf(2,4);
x2 = Tf(3,1); y2 = Tf(3,2); z2 = Tf(3,3); t2 = Tf(3,4);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Solve u6, u1, u2
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% For solution method, see AtlasSymbolicKinematics.m
% u6
C1 = t0*y0 + t1*y1 + t2*y2;
C2 = t0*x0 + t1*x1 + t2*x2;

u6(1) = atan(-C1/C2);
u6(2) = u6(1) + pi;

u6 = [u6(1) u6(1) u6(2) u6(2)];

% u2
for i = 1:2:3
   u2(i) = acos(y2*cos(u6(i)) + x2*sin(u6(i)));
   u2(i+1) = -u2(i);
end

% u1
for i = 1:4
   u1(i) = atan2( -( y1*cos(u6(i)) + x1*sin(u6(i)) ) / sin(u2(i)), ...
                  -( y0*cos(u6(i)) + x0*sin(u6(i)) ) / sin(u2(i)) );
end

% shift by constant offset
u2 = u2 + pi/2;
u1 = u1 - pi/2;

% extend
I = [1, 1, 2, 2, 3, 3, 4, 4];
u6 = u6(I);
u2 = u2(I);
u1 = u1(I);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Solve u3, u4, u5
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Geometric solution using Craig - Introduction to Robotics - 4.4

%% Constant DH parameters - link lengths
T = AtlasLegTransforms(0,0,0,0,0,0);
T01 = T{1};
T12 = T{2};
T23 = T{3};
T34 = T{4};
T45 = T{5};
T56 = T{6};

L1 = T23(1,4);
L2 = -T12(2,4);
L3 = T34(1,4);
L4 = T45(1,4);

%% Find variables in plane defined by frame 2
for i = 1:2:8
   T = AtlasLegTransforms(u1(i), u2(i), 0, 0, 0, u6(i));
   T2f = T{2}^-1 * T{1}^-1 * Tf;
   p2f = T2f(:,4);
   x = p2f(1) - L1;
   y = -p2f(3);
   
%    disp('>>>>>>>>>>>>>>>>>>>>')
%    T{1}
%    T{2}
%    T2f
%    u1(i)
%    u2(i)
%    x
%    y
%    acs = (x^2 + y^2 - L3^2 - L4^2) / (2*L3*L4)
   
   u4(i) = acos( (x^2 + y^2 - L3^2 - L4^2) / (2*L3*L4) );
   u4(i+1) = -u4(i);
end

for i = 1:8
   T = AtlasLegTransforms(u1(i), u2(i), 0, 0, 0, u6(i));
   T2f = T{2}^-1 * T{1}^-1 * Tf;
   p2f = T2f(:,4);
   x = p2f(1) - L1;
   y = -p2f(3);
   
   phi = atan2( T2f(1,3), T2f(3,3) );
   
   beta = atan2(y,x);
   psi = acos( (x^2 + y^2 + L3^2 - L4^2) / (2*L3*sqrt(x^2 + y^2)) );
   
   if u4(i) < 0
      u3(i) = beta + psi;
   else
      u3(i) = beta - psi;
   end
   
   u5(i) = phi - u4(i) - u3(i);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% u
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

u = [ u1 ;
      u2 ;
      u3 ;
      u4 ;
      u5 ;
      u6 ];
%u = real(u);

end

