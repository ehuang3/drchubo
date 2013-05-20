%% HuboRightLegIKEqns.m
% The "HuboRightLegIKEqns" script ...  TODO: Add description
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
%   Created 21-DEC-2012
%-------------------------------------------------------------------------------

%% Clear
ccc

%% Syms
syms q1 q2 q3 q4 q5 q6
syms l1 l2 l3 l4 l5 l6
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
t = [    sym(0), sym(-pi/2), sym(0), sym(0),    sym(0), sym(0)];
f = [ sym(pi/2), sym(-pi/2), sym(0), sym(0), sym(pi/2), sym(0)];
a = [    sym(0),     sym(0),     l4,     l5,    sym(0),     l6];
d = [    sym(0),     sym(0), sym(0), sym(0),    sym(0), sym(0)];

%% DH matrix
T = @(t,f,a,d) simplify([...
    cos(t), -sin(t)*cos(f),  sin(t)*sin(f), a*cos(t);...
    sin(t),  cos(t)*cos(f), -cos(t)*sin(f), a*sin(t);...
    sym(0),         sin(f),         cos(f),        d;...
    sym(0),         sym(0),         sym(0),   sym(1)]);
     
INV = @(H) simplify([H(1:3,1:3).' -H(1:3,1:3).'*H(1:3,4);sym(0),sym(0),sym(0),sym(1)]);

%% Coordinate transformations between joints
Ab0 = [sym(-1),  sym(0), sym(0),     l1;...
        sym(0), sym(-1), sym(0), sym(0);...
        sym(0),  sym(0), sym(1),    -l2;...
        sym(0),  sym(0), sym(0), sym(1)];
    
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
 

%% Write equations in terms of sins and coss
A60short = subs(expand(A60),....
    {sin(q1),sin(q2),sin(q3),sin(q4),sin(q5),sin(q6),...
     cos(q1),cos(q2),cos(q3),cos(q4),cos(q5),cos(q6)},...
    {S1,S2,S3,S4,S5,S6,...
     C1,C2,C3,C4,C5,C6});
 
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
fprintf(1,'========== Solving for q4 ==========\n');

A60simple = A60(1:3,4);
A60simple(1,1) = simplify((A60(1,4)+l6)/-cos(q6));
A60simple(1,1) = A60simple(1,1)*-cos(q6);
A60sum = simplify(sum(A60simple.^2));

T60simple = T60(1:3,4);
T60simple(1,1) = T60simple(1,1)+l6;
T60sum = simplify(sum(T60simple.^2));

C4 = solve(subs(A60sum,cos(q4),C4) == T60sum,C4);

fprintf(1,'C4 = %s;\n',char(C4));
fprintf(1,'q(4,i) = atan2(m(i,1)*real(sqrt(1-C4^2)),C4);\n');

fprintf(1,'====================================\n\n');


%% Solve for q5
fprintf(1,'========== Solving for q5 ==========\n');

fprintf(1,'S4 = sin(q(4,i));\n');
fprintf(1,'psi = atan2(S4*l4, C4*l4+l5);\n');
fprintf(1,'q(5,i) = atan2(-pz, m(i,2)*real(sqrt((px+l6)^2+py^2)))-psi;\n');

fprintf(1,'====================================\n\n');


%% Solve for q6
fprintf(1,'========== Solving for q6 ==========\n');

fprintf(1,'q(6,i) = atan2(py, -px-l6);\n');

fprintf(1,'====================================\n\n');

%% Solve for q2
fprintf(1,'========== Solving for q2 ==========\n');

S2 = solve(GL50short(2,3) == GR50short(2,3),S2);

fprintf(1,'S2 = %s;\n',char(S2));
fprintf(1,'q(2,i) = atan2(S2,m(i,3)*real(sqrt(1-S2^2)));\n');

fprintf(1,'====================================\n\n');

%% Solve for q1
fprintf(1,'========== Solving for q1 ==========\n');

fprintf(1,'q(1,i) = atan2(%s,%s);\n',char(GL50short(2,2)),char(GL50short(2,1)));

fprintf(1,'====================================\n\n');

%% Solve for q3
fprintf(1,'========== Solving for q3 ==========\n');

fprintf(1,'C2 = cos(q(2,i));\n');
fprintf(1,'q345 = atan2(%s,%s);\n',char(GL50short(3,3)/-C2),char(GL50short(1,3)/-C2));

fprintf(1,'q(3,i) = q345-q(4,i)-q(5,i);\n');

fprintf(1,'====================================\n\n');
