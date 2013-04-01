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
% For solution method, see AtlasSymbolicKinematics.m and Springer Handbook
% of Robotics - 1.7 Kinematics - Closed Form solutions
C1 = t0*y0 + t1*y1 + t2*y2;
C2 = t0*x0 + t1*x1 + t2*x2;

u6(1) = 2*atan( ( C2 + sqrt(C2^2 + C1^2) ) / C1 );
u6(2) = 2*atan( ( C2 - sqrt(C2^2 + C1^2) ) / C1 );

for i = 1:2
   u2(i) = acos(y2*cos(u6(i)) + x2*sin(u6(i)));
   u1(i) = asin( -( y1*cos(u6(i)) + x1*sin(u6(i)) ) / sin(u2(i)) );
end

u6 = [u6 u6(2:-1:1)]; % reverse second half for next loop
u2 = [u2 u2(2:-1:1)];
u1 = [u1 u1(2:-1:1)];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Solve u3, u4, u5
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Geometric solution using Craig - Introduction to Robotics - 4.4

% Constant DH parameters - link lengths
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

% Find variables in plane defined by frame 2
for i = 1:2:3
   T = AtlasLegTransforms(u1(i), u2(i), 0, 0, 0, u6(i));
   T2f = T{2}^-1 * T{1}^-1 * Tf;
   p2f = T2f(:,4);
   x = p2f(1) - L1;
   y = -p2f(3);
   
   u4(i) = acos( (x^2 + y^2 - L3^2 - L4^2) / (2*L3*L4) );
   u4(i+1) = -u4(i);
end

for i = 1:4
   T = AtlasLegTransforms(u1(i), u2(i), 0, 0, 0, u6(i));
   T2f = T{2}^-1 * T{1}^-1 * Tf;
   p2f = T2f(:,4);
   x = p2f(1) - L1;
   y = -p2f(3);
   
   phi = acos( T2f(3,3) );
   
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

end

