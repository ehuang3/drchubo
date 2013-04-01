function [ T ] = DH2Transform( DH_0 , DH_1 )
%DH2TRANSFORM 
% Returns transform from frame 0 to 1
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

T = [ cos(t1)        -sin(t1)          0        r0        ;
      sin(t1)*cos(a0) cos(t1)*cos(a0) -sin(a0) -sin(a0)*d1;
      sin(t1)*sin(a0) cos(t1)*sin(a0)  cos(a0)  cos(a0)*d1;
      0               0                0        1         ];

end

