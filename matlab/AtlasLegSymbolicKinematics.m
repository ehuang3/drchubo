%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% DH parameters
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear
clc

% general coordinates - joint angles
u = [ sym('u1') ;      % hip yaw
      sym('u2') ;      % hip roll
      sym('u3') ;      % hip pitch
      sym('u4') ;      % knee pitch
      sym('u5') ;      % ankle pitch
      sym('u6') ];     % ankle roll

l = [ sym('l1') ;      % hip pitch z
      sym('l2') ;      % hip pitch x
      sym('l3') ;      % hip - knee
      sym('l4') ];     % knee - foot

% REMEMBER, u1 actually is u1 + pi/2, u2 actually is u2 - pi/2

% frame 0 - atlas
DH(1) = struct('a', 0, 'alpha', 0, 'theta', 0, 'd', 0);
% frame 1 - hip yaw
DH(2) = struct('a', 0, 'alpha', pi/2, 'theta', u(1), 'd', 0);
% frame 2 - hip roll
DH(3) = struct('a', l(1), 'alpha', -pi/2, 'theta', u(2), 'd', l(2));
% frame 3 - hip pitch
DH(4) = struct('a', l(3), 'alpha', 0, 'theta', u(3), 'd', 0);
% frame 4 - knee pitch
DH(5) = struct('a', l(4), 'alpha', 0, 'theta', u(4), 'd', 0);
% frame 5 - ankle pitch
DH(6) = struct('a', 0, 'alpha', pi/2, 'theta', u(5), 'd', 0);
% frame 6 - ankle roll
DH(7) = struct('a', 0, 'alpha', 0, 'theta', u(6), 'd', 0);

T01 = DH2SymbolicTransform(DH(1), DH(2));
T12 = DH2SymbolicTransform(DH(2), DH(3));
T23 = DH2SymbolicTransform(DH(3), DH(4));
T34 = DH2SymbolicTransform(DH(4), DH(5));
T45 = DH2SymbolicTransform(DH(5), DH(6));
T56 = DH2SymbolicTransform(DH(6), DH(7));

p = [0 0 0 1]';

Tfoot = T01*T12*T23*T34*T45*T56;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Generic foot transform
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Tfoot = [ sym('x0') sym('y0') sym('z0') sym('t0');
          sym('x1') sym('y1') sym('z1') sym('t1');
          sym('x2') sym('y2') sym('z2') sym('t2');
          0         0         0         1        ];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Inverse equations
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

T03 = T01*T12*T23;
T5f = simplify(Tfoot * T56^-1 * T45^-1 * T34^-1);

z03 = T03(:,3)
z5f = T5f(:,3)

l0 = z03(1);
l1 = z03(2);
l2 = z03(3);

r0 = z5f(1);
r1 = z5f(2);
r2 = z5f(3);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Tf in frame 2
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

T2f = simplify(T12^-1 * T01^-1 * Tfoot);
T26 = T23*T34*T45*T56;

p2f = T2f(:,4)
p26 = T26(:,4)

l3 = p2f(2);

r3 = p26(2);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Eqs.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

z0 = -l0 == -r0
z1 = -l1 == -r1
z2 = l2 == r2
z3 = l3 == r3

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Leg solution
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

z = l3;
z = subs(z, 'cos(u1)', -r0); % subs(z, -l0, -r0)
z = subs(z, 'sin(u2)', 1);
z = subs(z, 'sin(u1)', -r1); % subs(z, -l1, -r1)
z = subs(z, l2, r2);
z = expand(z)
