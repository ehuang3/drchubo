%% Clear
ccc

%% Syms
syms q1 q2 h1 h2

Tn0 = [sym(1)   sym(0)  sym(0) sym(0);
       sym(0)   sym(1)  sym(0) sym(0);
       sym(0)   sym(0)  sym(1) h1;
       sym(0)   sym(0)  sym(0) sym(1)];

Tn1 = [cos(q1) -sin(q1) sym(0) sym(0);
       sin(q1)  cos(q1) sym(0) sym(0);
       sym(0)   sym(0)  sym(1) sym(0);
       sym(0)   sym(0)  sym(0) sym(1)];
   
T12 = [cos(q2) -sin(q2) sym(0) sym(0);
       sym(0)   sym(0)  sym(1) sym(0);
      -sin(q2) -cos(q1) sym(0) sym(0);
       sym(0)   sym(0)  sym(0) sym(1)];
   
T2h = [sym(1)   sym(0)  sym(0) sym(0);
       sym(0)   sym(0) -sym(1) -h2;
       sym(0)   sym(1)  sym(0) sym(0);
       sym(0)   sym(0)  sym(0) sym(1)];
   
Tnh = Tn0*Tn1*T12*T2h