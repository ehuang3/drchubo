%% HuboRightArmIKEqns.m
% The "HuboRightArmIKEqns" script uses symbolic math to solve for the
% analytical solution to the inverse kinematics of Hubo.
%
% NOTES:
%
% NECESSARY FILES AND/OR PACKAGES: TODO: Add necessary files
%   +somePackage, someFile.m
%
% SEE ALSO: TODO: Add see alsos
%    relatedFunction1 | relatedFunction2
%
% AUTHOR:
%    Rowland O'Flaherty (www.rowlandoflaherty.com)
%
% VERSION: 
%   Created 09-JAN-2013
%-------------------------------------------------------------------------------

%% Clear
ccc

%% Syms
syms q1 q2 q3 q4
syms l1 l2 l3

syms nx sx ax px
syms ny sy ay py
syms nz sz az pz

syms inx isx iax ipx
syms iny isy iay ipy
syms inz isz iaz ipz

syms C1 C2 C3 C4 C5 C6
syms S1 S2 S3 S4 S5 S6

q = [q1 q2 q3 q4];

T05 = [nx sx ax px;...
       ny sy ay py;...
       nz sz az pz;...
       sym(0) sym(0) sym(0) sym(1)];

T05p = T05(:,4);

T50 = [inx isx iax ipx;...
       iny isy iay ipy;...
       inz isz iaz ipz;...
       sym(0) sym(0) sym(0) sym(1)];
   
T50p = T50(:,4);


%% DH parameters
t = [ sym(pi/2),sym(-pi/2), sym(pi/2),    sym(0)];
f = [ sym(pi/2), sym(pi/2),sym(-pi/2), sym(pi/2)];
a = [    sym(0),    sym(0),    sym(0),    sym(0)];
d = [    sym(0),    sym(0),  sym(-l2),    sym(0)];

wrist = [...
        sym(1)   sym(0)   sym(0)   sym(0)
        sym(0)   sym(1)   sym(0)   sym(0)
        sym(0)   sym(0)   sym(1) sym(-l3)
        sym(0)   sym(0)   sym(0)   sym(1)];
    
wristInv = wrist;

%% DH matrix
T = @(t,f,a,d) simplify([...
    cos(t), -sin(t)*cos(f),  sin(t)*sin(f), a*cos(t);...
    sin(t),  cos(t)*cos(f), -cos(t)*sin(f), a*sin(t);...
    sym(0),         sin(f),         cos(f),        d;...
    sym(0),         sym(0),         sym(0),   sym(1)]);
     
INV = @(H) simplify([H(1:3,1:3).' -H(1:3,1:3).'*H(1:3,4);sym(0),sym(0),sym(0),sym(1)]);

%% Coordinate transformations between joints
i = 1; A01 = T(t(i)+q(i),f(i),a(i),d(i));
i = 2; A12 = T(t(i)+q(i),f(i),a(i),d(i));
i = 3; A23 = T(t(i)+q(i),f(i),a(i),d(i));
i = 4; A34 = T(t(i)+q(i),f(i),a(i),d(i));
A45 = wrist;

A10 = INV(A01);
A21 = INV(A12);
A32 = INV(A23);
A43 = INV(A34);
A54 = INV(A45);

A05 = simplify(A01*A12*A23*A34*A45);
A05short = subs(A05,....
    {sin(q1),sin(q2),sin(q3),sin(q4),...
     cos(q1),cos(q2),cos(q3),cos(q4)},...
    {S1,S2,S3,S4,...
     C1,C2,C3,C4});
 
A50 = simplify(A54*A43*A32*A21*A10);
A50short = subs(A50,.... 
    {sin(q1),sin(q2),sin(q3),sin(q4),...
     cos(q1),cos(q2),cos(q3),cos(q4)},...
    {S1,S2,S3,S4,...
     C1,C2,C3,C4}); 
 
A03 = simplify(A01*A12*A23);
A03short = subs(A03,....
    {sin(q1),sin(q2),sin(q3),sin(q4),...
     cos(q1),cos(q2),cos(q3),cos(q4)},...
    {S1,S2,S3,S4,...
     C1,C2,C3,C4});
 
T03 = simplify(T05*A54*A43);
T03short = subs(T03,....
    {sin(q1),sin(q2),sin(q3),sin(q4),...
     cos(q1),cos(q2),cos(q3),cos(q4)},...
    {S1,S2,S3,S4,...
     C1,C2,C3,C4});

%% Use inverse transform method going forward
clc

% 0
% T05p(1:3,1)
% A05short(1:3,4)

% 1
T15p = simplify(A10*T05p);
A15 = simplify(A10*A05);
 
T15pshort = subs(T15p,....
    {sin(q1),sin(q2),sin(q3),sin(q4),...
     cos(q1),cos(q2),cos(q3),cos(q4)},...
    {S1,S2,S3,S4,...
     C1,C2,C3,C4});
A15short = subs(A15,.... 
    {sin(q1),sin(q2),sin(q3),sin(q4),...
     cos(q1),cos(q2),cos(q3),cos(q4)},...
    {S1,S2,S3,S4,...
     C1,C2,C3,C4});
 
% T15pshort(1:3,1)
% A15short(1:3,4)


% 2
T25p = simplify(A21*A10*T05p);
A25 = simplify(A21*A10*A05);
 
T25pshort = subs(T25p,....
    {sin(q1),sin(q2),sin(q3),sin(q4),...
     cos(q1),cos(q2),cos(q3),cos(q4)},...
    {S1,S2,S3,S4,...
     C1,C2,C3,C4});
A25short = subs(A25,.... 
    {sin(q1),sin(q2),sin(q3),sin(q4),...
     cos(q1),cos(q2),cos(q3),cos(q4)},...
    {S1,S2,S3,S4,...
     C1,C2,C3,C4});
 
% T25pshort(1:3,1)
% A25short(1:3,4)


% 3
T35p = simplify(A32*A21*A10*T05p);
A35 = simplify(A32*A21*A10*A05);
 
T35pshort = subs(T35p,....
    {sin(q1),sin(q2),sin(q3),sin(q4),...
     cos(q1),cos(q2),cos(q3),cos(q4)},...
    {S1,S2,S3,S4,...
     C1,C2,C3,C4});
A35short = subs(A35,.... 
    {sin(q1),sin(q2),sin(q3),sin(q4),...
     cos(q1),cos(q2),cos(q3),cos(q4)},...
    {S1,S2,S3,S4,...
     C1,C2,C3,C4});
 
% T35pshort(1:3,1)
% A35short(1:3,4)

% 4
T45p = simplify(A43*A32*A21*A10*T05p);
 
T45pshort = subs(T45p,....
    {sin(q1),sin(q2),sin(q3),sin(q4),...
     cos(q1),cos(q2),cos(q3),cos(q4)},...
    {S1,S2,S3,S4,...
     C1,C2,C3,C4});
A45short = subs(A45,.... 
    {sin(q1),sin(q2),sin(q3),sin(q4),...
     cos(q1),cos(q2),cos(q3),cos(q4)},...
    {S1,S2,S3,S4,...
     C1,C2,C3,C4});
 
% T45pshort(1:3,1)
% A45short(1:3,4)

%% Use inverse transform method going backward
% 
% % 5
% T50p(1:3,1)
% A50short(1:3,4)
%
% 4
T40p = simplify(A45*T50p);
A40 = simplify(A45*A05);
 
T40pshort = subs(T40p,....
    {sin(q1),sin(q2),sin(q3),sin(q4),...
     cos(q1),cos(q2),cos(q3),cos(q4)},...
    {S1,S2,S3,S4,...
     C1,C2,C3,C4});
A40short = subs(A40,.... 
    {sin(q1),sin(q2),sin(q3),sin(q4),...
     cos(q1),cos(q2),cos(q3),cos(q4)},...
    {S1,S2,S3,S4,...
     C1,C2,C3,C4});
 
% T40pshort(1:3,1)
% A40short(1:3,4)


% 3
T30p = simplify(A34*A45*T50p);
A30 = simplify(A34*A45*A05);
 
T30pshort = subs(T30p,....
    {sin(q1),sin(q2),sin(q3),sin(q4),...
     cos(q1),cos(q2),cos(q3),cos(q4)},...
    {S1,S2,S3,S4,...
     C1,C2,C3,C4});
A30short = subs(A30,.... 
    {sin(q1),sin(q2),sin(q3),sin(q4),...
     cos(q1),cos(q2),cos(q3),cos(q4)},...
    {S1,S2,S3,S4,...
     C1,C2,C3,C4});
 
% T30pshort(1:3,1)
% A30short(1:3,4)


% % 2
% T20p = simplify(A23*A34*A45*T50p);
% A20 = simplify(A23*A34*A45*A05);
%  
% T20pshort = subs(T20p,....
%     {sin(q1),sin(q2),sin(q3),sin(q4),...
%      cos(q1),cos(q2),cos(q3),cos(q4)},...
%     {S1,S2,S3,S4,...
%      C1,C2,C3,C4});
% A20short = subs(A20,.... 
%     {sin(q1),sin(q2),sin(q3),sin(q4),...
%      cos(q1),cos(q2),cos(q3),cos(q4)},...
%     {S1,S2,S3,S4,...
%      C1,C2,C3,C4});
%  
% T20pshort(1:3,1)
% A20short(1:3,4)
% 
% % 1
% T10p = simplify(A12*A23*A34*A45*T50p);
% A10 = simplify(A12*A23*A34*A45*A05);
%  
% T10pshort = subs(T10p,....
%     {sin(q1),sin(q2),sin(q3),sin(q4),...
%      cos(q1),cos(q2),cos(q3),cos(q4)},...
%     {S1,S2,S3,S4,...
%      C1,C2,C3,C4});
% A10short = subs(A10,.... 
%     {sin(q1),sin(q2),sin(q3),sin(q4),...
%      cos(q1),cos(q2),cos(q3),cos(q4)},...
%     {S1,S2,S3,S4,...
%      C1,C2,C3,C4});
%  
% T10pshort(1:3,1)
% A10short(1:3,4)

%% Initilize
fprintf(1,'================= Initialize ================\n');

fprintf(1,'q = nan(4,2);\n');
fprintf(1,'m = [...\n');
fprintf(1,'   1;...\n');
fprintf(1,'  -1];\n');

fprintf(1,'=============================================\n\n');
 
%% Solve for q4
fprintf(1,'=============== Solving for q4 ==============\n');

S4s = solve(A50short(1,4) == T50(1,4),S4);

fprintf(1,'S4 = %s;\n',char(S4s));
fprintf(1,'q(4,i) = atan2(S4,m(i,1)*real(sqrt(1-S4^2)));\n');
fprintf(1,'C4 = cos(q(4,i));\n');

fprintf(1,'=============================================\n\n');

%% Solve for q1
fprintf(1,'=============== Solving for q1 ==============\n');

% fprintf(1,'q(1,i) = q1;\n');
% fprintf(1,'S1 = sin(q(1,i));\n');
% fprintf(1,'C1 = cos(q(1,i));\n');

fprintf(1,'=============================================\n\n');

%% Solve for q2
fprintf(1,'=============== Solving for q2 ==============\n');

S2s = solve(A03short(3,4) == T03short(3,4),S2);

fprintf(1,'S2 = %s;\n',char(S2s));
fprintf(1,'q(2,i) = atan2(S2,m(i,1)*real(sqrt(1-S2^2)));\n');
fprintf(1,'C2 = cos(q(2,i));\n');

% Q23s = solve(T25pshort(3,1) == A25short(3,4), T25pshort(1,1) == A25short(1,4), T15pshort(2,1) == A15short(2,4), S2, C2, S3);
% 
% S2s = subs(simple(subs(Q23s.S2,.... 
%     {S1,S2,S3,S4,...
%      C1,C2,C3,C4},...
%     {sin(q1),sin(q2),sin(q3),sin(q4),...
%      cos(q1),cos(q2),cos(q3),cos(q4)})),.... 
%     {sin(q1),sin(q2),sin(q3),sin(q4),...
%      cos(q1),cos(q2),cos(q3),cos(q4)},...
%     {S1,S2,S3,S4,...
%      C1,C2,C3,C4});
%  
% C2s = subs(simple(subs(Q23s.C2,.... 
%     {S1,S2,S3,S4,...
%      C1,C2,C3,C4},...
%     {sin(q1),sin(q2),sin(q3),sin(q4),...
%      cos(q1),cos(q2),cos(q3),cos(q4)})),.... 
%     {sin(q1),sin(q2),sin(q3),sin(q4),...
%      cos(q1),cos(q2),cos(q3),cos(q4)},...
%     {S1,S2,S3,S4,...
%      C1,C2,C3,C4});
% 
% fprintf(1,'S2 = %s;\n',char(S2s(1)));
% fprintf(1,'S2 = %s;\n',char(S2s(2)));
% fprintf(1,'C2 = %s;\n',char(C2s(1)));
% fprintf(1,'C2 = %s;\n',char(C2s(2)));
% fprintf(1,'q(2,i) = atan2(S2,C2);\n');

fprintf(1,'=============================================\n\n');


%% Solve for q3
fprintf(1,'=============== Solving for q3 ==============\n');
Q3s = solve(T45pshort(1,1) == A45short(1,4),T45pshort(2,1) == A45short(2,4),S3,C3);
S3s = Q3s.S3;
C3s = Q3s.C3;

fprintf(1,'S3 = %s;\n',char(S3s));
fprintf(1,'C3 = %s;\n',char(C3s));
fprintf(1,'q(3,i) = atan2(S3,C3);\n');

fprintf(1,'=============================================\n\n');



%% Case 1: q4 == 0
fprintf(1,'=============== Case 1: q4 == 0 =============\n');

% fprintf(1,'if abs(q(4,i)) <= zeroSize\n');
% 
% S6s = solve(T60(2,4) == subs(subs(A60(2,4),q4,0),{cos(q6),sin(q6)},{C6,S6}),S6);
% C6s = solve(T60(1,4) == subs(subs(A60(1,4),q4,0),{cos(q6),sin(q6)},{C6,S6}),C6);
% 
% fprintf(1,'\tS6 = %s;\n',char(S6s));
% fprintf(1,'\tC6 = %s;\n',char(C6s));
% fprintf(1,'\tq(6,i) = atan2(S6,C6);\n');
% 
% % simplify(subs(simplify(subs(GR50short(1,3),{S4,C4},{0,1})/C2),{S3,S5,C3,C5},{sin(q3),sin(q5),cos(q3),cos(q5)}))
% % simplify(subs(simplify(subs(GR50short(3,3),{S4,C4},{0,1})/C2),{S3,S5,C3,C5},{sin(q3),sin(q5),cos(q3),cos(q5)}))
% 
% S3p5 = GL50short(1,3);
% C3p5 = GL50short(3,3);
% 
% fprintf(1,'\tqT = atan2(%s,%s);\n',char(S3p5),char(C3p5));
% fprintf(1,'\tif C2 < 0, qT = qT + pi; end\n');
% fprintf(1,'\tq(5,i) = wrapToPi(qT - q(3,i));\n');
% fprintf(1,'end\n');

fprintf(1,'=============================================\n\n');


%% Case 2a: q2 == pi/2
fprintf(1,'============ Case 2a: q2 == pi/2 ============\n');

% % -simplify(subs(simplify(subs(GR50short(2,2),{S2,C2},{1,0})/S4),{S1,S3,C1,C3},{sin(q1),sin(q3),cos(q1),cos(q3)}))
% % -simplify(subs(simplify(subs(GR50short(2,1),{S2,C2},{1,0})/S4),{S1,S3,C1,C3},{sin(q1),sin(q3),cos(q1),cos(q3)}))
% 
% S1m3 = -GL50short(2,2);
% C1m3 = -GL50short(2,1);
% 
% fprintf(1,'if abs(q(2,i)) - pi/2 <= zeroSize\n');
% fprintf(1,'\tqT = atan2(%s,%s);\n',char(S1m3),char(C1m3));
% fprintf(1,'\tif S4 < 0, qT = qT + pi; end\n');
% fprintf(1,'\tq(1,i) = wrapToPi(qT + q(3,i));\n');
% fprintf(1,'end\n');

fprintf(1,'=============================================\n\n');


%% Case 2b: q2 == -pi/2
fprintf(1,'============= Case 2b: q2 == -pi/2 ==========\n');

% % -simplify(subs(simplify(subs(GR50short(2,2),{S2,C2},{-1,0})/S4),{S1,S3,C1,C3},{sin(q1),sin(q3),cos(q1),cos(q3)}))
% % -simplify(subs(simplify(subs(GR50short(2,1),{S2,C2},{-1,0})/S4),{S1,S3,C1,C3},{sin(q1),sin(q3),cos(q1),cos(q3)}))
% 
% S1p3 = -GL50short(2,2);
% C1p3 = -GL50short(2,1);
% 
% fprintf(1,'if abs(q(2,i)) + pi/2 <= zeroSize\n');
% fprintf(1,'\tqT = atan2(%s,%s);\n',char(S1p3),char(C1p3));
% fprintf(1,'\tif S4 < 0, qT = qT + pi; end\n');
% fprintf(1,'\tq(1,i) = wrapToPi(qT - q(3,i));\n');
% fprintf(1,'end\n');

fprintf(1,'=============================================\n\n');


%% Case 3a: q4 == 0 and q2 == pi/2
fprintf(1,'====== Case 3a: q4 == 0 and q2 == pi/2 ======\n');

% % simplify(subs(subs(GR50short(3,1),{S2,C2,S4,C4},{1,0,0,1}),{S1,S3,S5,C1,C3,C5},{sin(q1),sin(q3),sin(q5),cos(q1),cos(q3),cos(q5)}))
% % -simplify(subs(subs(GR50short(3,2),{S2,C2,S4,C4},{1,0,0,1}),{S1,S3,S5,C1,C3,C5},{sin(q1),sin(q3),sin(q5),cos(q1),cos(q3),cos(q5)}))
% 
% S1m3m5 = GL50short(3,1);
% C1m3m5 = -GL50short(3,2);
% 
% fprintf(1,'if abs(q(4,i)) == zeroSize && q(2,i) - pi/2 <= zeroSize\n');
% fprintf(1,'\tqT = atan2(%s,%s);\n',char(S1m3m5),char(C1m3m5));
% fprintf(1,'\tq(5,i) = wrapToPi(q(1,i) - q(3,i) - qT);\n');
% fprintf(1,'end\n');

fprintf(1,'=============================================\n\n');


%% Case 3b: q4 == 0 and q2 == -pi/2
fprintf(1,'====== Case 3b: q4 == 0 and q2 == -pi/2 ======\n');

% % -simplify(subs(subs(GR50short(3,1),{S2,C2,S4,C4},{-1,0,0,1}),{S1,S3,S5,C1,C3,C5},{sin(q1),sin(q3),sin(q5),cos(q1),cos(q3),cos(q5)}))
% % simplify(subs(subs(GR50short(3,2),{S2,C2,S4,C4},{-1,0,0,1}),{S1,S3,S5,C1,C3,C5},{sin(q1),sin(q3),sin(q5),cos(q1),cos(q3),cos(q5)}))
% 
% S1p3p5 = -GL50short(3,1);
% C1p3p5 = GL50short(3,2);
% 
% fprintf(1,'if abs(q(4,i)) == zeroSize && q(2,i) + pi/2 <= zeroSize\n');
% fprintf(1,'\tqT = atan2(%s,%s);\n',char(S1p3p5),char(C1p3p5));
% fprintf(1,'\tq(5,i) = wrapToPi(qT - q(1,i) - q(3,i));\n');
% fprintf(1,'end\n');

fprintf(1,'=============================================\n\n');

