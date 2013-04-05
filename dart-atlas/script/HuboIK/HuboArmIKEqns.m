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
%   Created 16-DEC-2012
%-------------------------------------------------------------------------------

%% Clear
%%ccc
clear
clc

%% Syms
syms q1 q2 q3 q4 q5 q6
syms l1 l2 l3 l4
syms nx sx ax px
syms ny sy ay py
syms nz sz az pz

syms r
syms C1 C2 C3 C4 C5 C6
syms S1 S2 S3 S4 S5 S6

syms C6_2 S6_2

q = [q1 q2 q3 q4 q5 q6];

T60 = [nx sx ax px;...
       ny sy ay py;...
       nz sz az pz;...
       sym(0) sym(0) sym(0) sym(1)];


%% DH parameters
% t = [sym(-pi/2), sym(pi/2), sym(pi/2),    sym(0),    sym(0), sym(pi/2)];
% f = [ sym(pi/2),sym(-pi/2), sym(pi/2), sym(pi/2), sym(pi/2),    sym(0)];
% a = [    sym(0),    sym(0),    sym(0),    sym(0),    sym(0),   sym(l4)];
% d = [    sym(0),    sym(0),  sym(-l2),    sym(0),   sym(l3),    sym(0)];

t = [ sym(pi/2),sym(-pi/2), sym(pi/2),    sym(0),    sym(0), sym(pi/2)];
f = [ sym(pi/2), sym(pi/2),sym(-pi/2), sym(pi/2),sym(-pi/2),    sym(0)];
a = [    sym(0),    sym(0),    sym(0),    sym(0),    sym(0),   sym(l4)];
d = [    sym(0),    sym(0),  sym(-l2),    sym(0),  sym(-l3),    sym(0)];

%% DH matrix
T = @(t,f,a,d) simplify([...
    cos(t), -sin(t)*cos(f),  sin(t)*sin(f), a*cos(t);...
    sin(t),  cos(t)*cos(f), -cos(t)*sin(f), a*sin(t);...
    sym(0),         sin(f),         cos(f),        d;...
    sym(0),         sym(0),         sym(0),   sym(1)]);
     
INV = @(H) simplify([H(1:3,1:3).' -H(1:3,1:3).'*H(1:3,4);sym(0),sym(0),sym(0),sym(1)]);

%% Coordinate transformations between joints
Ab0 = [sym(0), sym(0), sym(1),     l1;...
       sym(1), sym(0), sym(0), sym(0);...
       sym(0), sym(1), sym(0), sym(0);...
       sym(0), sym(0), sym(0), sym(1)];

i = 1; A01 = T(t(i)+q(i),f(i),a(i),d(i));
i = 2; A12 = T(t(i)+q(i),f(i),a(i),d(i));
i = 3; A23 = T(t(i)+q(i),f(i),a(i),d(i));
i = 4; A34 = T(t(i)+q(i),f(i),a(i),d(i));
i = 5; A45 = T(t(i)+q(i),f(i),a(i),d(i));
i = 6; A56 = T(t(i)+q(i),f(i),a(i),d(i));

A10 = INV(A01);
A21 = INV(A12);
A32 = INV(A23);
A43 = INV(A34);
A54 = INV(A45);
A65 = INV(A56);

A06 = simplify(A01*A12*A23*A34*A45*A56);
A60 = simplify(A65*A54*A43*A32*A21*A10);

%% Use inverse transform method (base to wrist pitch)
GL50 = simplify(A56*T60);
GR50 = simplify(A56*A60);

GL50(1:3,4);
GR50(1:3,4);
 
GL50short = subs(GL50,....
    {sin(q1),sin(q2),sin(q3),sin(q4),sin(q5),sin(q6),...
     cos(q1),cos(q2),cos(q3),cos(q4),cos(q5),cos(q6)},...
    {S1,S2,S3,S4,S5,S6,...
     C1,C2,C3,C4,C5,C6});
GR50short = subs(GR50,.... 
    {sin(q1),sin(q2),sin(q3),sin(q4),sin(q5),sin(q6),...
     cos(q1),cos(q2),cos(q3),cos(q4),cos(q5),cos(q6)},...
    {S1,S2,S3,S4,S5,S6,...
     C1,C2,C3,C4,C5,C6});
 

%% Use inverse transform method (base to shoulder yaw)
GL30 = simplify(A34*A45*A56*T60);
GR30 = simplify(A34*A45*A56*A60);

GL30short = subs(GL30,....
    {sin(q1),sin(q2),sin(q3),sin(q4),sin(q5),sin(q6),...
     cos(q1),cos(q2),cos(q3),cos(q4),cos(q5),cos(q6)},...
    {S1,S2,S3,S4,S5,S6,...
     C1,C2,C3,C4,C5,C6});
GR30short = subs(GR30,.... 
    {sin(q1),sin(q2),sin(q3),sin(q4),sin(q5),sin(q6),...
     cos(q1),cos(q2),cos(q3),cos(q4),cos(q5),cos(q6)},...
    {S1,S2,S3,S4,S5,S6,...
     C1,C2,C3,C4,C5,C6});
 
%% Initilize
fprintf(1,'================= Initialize ================\n');

fprintf(1,'q = nan(6,8);\n');
fprintf(1,'m = [...\n');
fprintf(1,'    1  1  1;...\n');
fprintf(1,'    1  1 -1;...\n');
fprintf(1,'    1 -1  1;...\n');
fprintf(1,'    1 -1 -1;...\n');
fprintf(1,'   -1  1  1;...\n');
fprintf(1,'   -1  1 -1;...\n');
fprintf(1,'   -1 -1  1;...\n');
fprintf(1,'   -1 -1 -1];\n');

fprintf(1,'=============================================\n\n');
 
%% Solve for q4 in terms of C4
fprintf(1,'=============== Solving for q4 ==============\n');

GL50sum = simplify(sum(GL50(1:3,4).^2));
GR50sum = simplify(sum(GR50(1:3,4).^2));
GR50sum = subs(GR50sum,cos(q4),C4);

C4s = simplify(solve(GL50sum == GR50sum,C4));

fprintf(1,'C4 = %s;\n',char(C4s));
fprintf(1,'q(4,i) = atan2(m(i,1)*sqrt(1-C4^2),C4);\n');

fprintf(1,'=============================================\n\n');

%% Solve for q5 in terms of S5
fprintf(1,'=============== Solving for q5 ==============\n');

S5s = simplify(solve(GL50short(3,4) == GR50short(3,4),S5));

fprintf(1,'S4 = sin(q4);\n');
fprintf(1,'S5 = %s;\n',char(S5s));
fprintf(1,'q(5,i) = atan2(S5,m(i,2)*sqrt(1-S5^2));\n');

fprintf(1,'=============================================\n\n');

%% Solve for q6 in terms of S6 and C6
fprintf(1,'=============== Solving for q6 ==============\n');

S6s = simplify(solve(GL50short(1,4) == GR50short(1,4),S6));
C6s = simplify(solve(subs(GL50short(2,4),S6,S6s) == GR50short(2,4),C6));

S6s = subs(S6s,C6,C6s);

fprintf(1,'C5 = cos(q5);\n');
fprintf(1,'S6 = %s;\n',char(S6s));
fprintf(1,'C6 = %s;\n',char(C6s));
fprintf(1,'q(6,i) = atan2(S6,C6);\n');

fprintf(1,'=============================================\n\n');

%% Solve for q2
fprintf(1,'=============== Solving for q2 ==============\n');

S2s = simplify(solve(GL30short(2,3) == GR30short(2,3),S2));

fprintf(1,'S2 = %s;\n',char(S2s));
fprintf(1,'q(2,i) = atan2(S2,m(i,3)*sqrt(1-S2^2));\n');

fprintf(1,'=============================================\n\n');

%% Solve for q3
fprintf(1,'=============== Solving for q3 ==============\n');

S3C2s = simplify(solve(GL30short(1,3) == GR30short(1,3),S3))*C2;
C3C2s = simplify(solve(GL30short(3,3) == GR30short(3,3),C3))*C2;

fprintf(1,'S3C2 = %s;\n',char(S3C2s));
fprintf(1,'C3C2 = %s;\n',char(C3C2s));
fprintf(1,'q(3,i) = atan2(S3C2,C3C2);\n');

fprintf(1,'=============================================\n\n');

%% Solve for q1
fprintf(1,'=============== Solving for q1 ==============\n');

S1C2s = simplify(solve(GL30short(2,1) == GR30short(2,1),S1))*C2;
C1C2s = simplify(solve(GL30short(2,2) == GR30short(2,2),C1))*C2;

fprintf(1,'S1C2 = %s;\n',char(S1C2s));
fprintf(1,'C1C2 = %s;\n',char(C1C2s));
fprintf(1,'q(1,i) = atan2(S1C2,C1C2);\n');

fprintf(1,'=============================================\n\n');

%% Case 1: q4 == 0
fprintf(1,'=============== Case 1: q4 == 0 =============\n');

fprintf(1,'if abs(q(4,i)) <= zeroSize\n');

S6s = solve(T60(2,4) == subs(subs(A60(2,4),q4,0),{cos(q6),sin(q6)},{C6,S6}),S6);
C6s = solve(T60(1,4) == subs(subs(A60(1,4),q4,0),{cos(q6),sin(q6)},{C6,S6}),C6);

fprintf(1,'\tS6 = %s;\n',char(S6s));
fprintf(1,'\tC6 = %s;\n',char(C6s));
fprintf(1,'\tq(6,i) = atan2(S6,C6);\n');

% simplify(subs(simplify(subs(GR50short(1,3),{S4,C4},{0,1})/C2),{S3,S5,C3,C5},{sin(q3),sin(q5),cos(q3),cos(q5)}))
% simplify(subs(simplify(subs(GR50short(3,3),{S4,C4},{0,1})/C2),{S3,S5,C3,C5},{sin(q3),sin(q5),cos(q3),cos(q5)}))

S3p5 = GL50short(1,3);
C3p5 = GL50short(3,3);

fprintf(1,'\tqT = atan2(%s,%s);\n',char(S3p5),char(C3p5));
fprintf(1,'\tif C2 < 0, qT = qT + pi; end\n');
fprintf(1,'\tq(5,i) = wrapToPi(qT - q(3,i));\n');
fprintf(1,'end\n');

fprintf(1,'=============================================\n\n');


%% Case 2a: q2 == pi/2
fprintf(1,'============ Case 2a: q2 == pi/2 ============\n');

% -simplify(subs(simplify(subs(GR50short(2,2),{S2,C2},{1,0})/S4),{S1,S3,C1,C3},{sin(q1),sin(q3),cos(q1),cos(q3)}))
% -simplify(subs(simplify(subs(GR50short(2,1),{S2,C2},{1,0})/S4),{S1,S3,C1,C3},{sin(q1),sin(q3),cos(q1),cos(q3)}))

S1m3 = -GL50short(2,2);
C1m3 = -GL50short(2,1);

fprintf(1,'if abs(q(2,i)) - pi/2 <= zeroSize\n');
fprintf(1,'\tqT = atan2(%s,%s);\n',char(S1m3),char(C1m3));
fprintf(1,'\tif S4 < 0, qT = qT + pi; end\n');
fprintf(1,'\tq(1,i) = wrapToPi(qT + q(3,i));\n');
fprintf(1,'end\n');

fprintf(1,'=============================================\n\n');


%% Case 2b: q2 == -pi/2
fprintf(1,'============= Case 2b: q2 == -pi/2 ==========\n');

% -simplify(subs(simplify(subs(GR50short(2,2),{S2,C2},{-1,0})/S4),{S1,S3,C1,C3},{sin(q1),sin(q3),cos(q1),cos(q3)}))
% -simplify(subs(simplify(subs(GR50short(2,1),{S2,C2},{-1,0})/S4),{S1,S3,C1,C3},{sin(q1),sin(q3),cos(q1),cos(q3)}))

S1p3 = -GL50short(2,2);
C1p3 = -GL50short(2,1);

fprintf(1,'if abs(q(2,i)) + pi/2 <= zeroSize\n');
fprintf(1,'\tqT = atan2(%s,%s);\n',char(S1p3),char(C1p3));
fprintf(1,'\tif S4 < 0, qT = qT + pi; end\n');
fprintf(1,'\tq(1,i) = wrapToPi(qT - q(3,i));\n');
fprintf(1,'end\n');

fprintf(1,'=============================================\n\n');


%% Case 3a: q4 == 0 and q2 == pi/2
fprintf(1,'====== Case 3a: q4 == 0 and q2 == pi/2 ======\n');

% simplify(subs(subs(GR50short(3,1),{S2,C2,S4,C4},{1,0,0,1}),{S1,S3,S5,C1,C3,C5},{sin(q1),sin(q3),sin(q5),cos(q1),cos(q3),cos(q5)}))
% -simplify(subs(subs(GR50short(3,2),{S2,C2,S4,C4},{1,0,0,1}),{S1,S3,S5,C1,C3,C5},{sin(q1),sin(q3),sin(q5),cos(q1),cos(q3),cos(q5)}))

S1m3m5 = GL50short(3,1);
C1m3m5 = -GL50short(3,2);

fprintf(1,'if abs(q(4,i)) == zeroSize && q(2,i) - pi/2 <= zeroSize\n');
fprintf(1,'\tqT = atan2(%s,%s);\n',char(S1m3m5),char(C1m3m5));
fprintf(1,'\tq(5,i) = wrapToPi(q(1,i) - q(3,i) - qT);\n');
fprintf(1,'end\n');

fprintf(1,'=============================================\n\n');


%% Case 3b: q4 == 0 and q2 == -pi/2
fprintf(1,'====== Case 3b: q4 == 0 and q2 == -pi/2 ======\n');

% -simplify(subs(subs(GR50short(3,1),{S2,C2,S4,C4},{-1,0,0,1}),{S1,S3,S5,C1,C3,C5},{sin(q1),sin(q3),sin(q5),cos(q1),cos(q3),cos(q5)}))
% simplify(subs(subs(GR50short(3,2),{S2,C2,S4,C4},{-1,0,0,1}),{S1,S3,S5,C1,C3,C5},{sin(q1),sin(q3),sin(q5),cos(q1),cos(q3),cos(q5)}))

S1p3p5 = -GL50short(3,1);
C1p3p5 = GL50short(3,2);

fprintf(1,'if abs(q(4,i)) == zeroSize && q(2,i) + pi/2 <= zeroSize\n');
fprintf(1,'\tqT = atan2(%s,%s);\n',char(S1p3p5),char(C1p3p5));
fprintf(1,'\tq(5,i) = wrapToPi(qT - q(1,i) - q(3,i));\n');
fprintf(1,'end\n');

fprintf(1,'=============================================\n\n');

