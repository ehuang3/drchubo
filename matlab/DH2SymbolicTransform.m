function [ T ] = DH2SymbolicTransform( DH_0 , DH_1 )
%DH2TRANSFORM 
% Returns symbolic transform from frame 0 to 1
% 
% Taken from Craig - Introduction to Robotics

r0 = DH_0.a;
a0 = DH_0.alpha;
t0 = DH_0.theta;
d0 = DH_0.d;

r1 = DH_1.a;
a1 = DH_1.alpha;
t1 = DH_1.theta;
d1 = DH_1.d;

% cos(pi/2) is not 0, therefore, I must zero it
if a0 == pi/2 || a0 == -pi/2
   ca0 = sym('0');
else
   ca0 = cos(a0);
end

T = [ cos(t1)        -sin(t1)          0        r0        ;
      sin(t1)*ca0     cos(t1)*ca0     -sin(a0) -sin(a0)*d1;
      sin(t1)*sin(a0) cos(t1)*sin(a0)  ca0      ca0*d1    ;
      0               0                0        1         ];

end