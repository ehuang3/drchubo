function [ T ] = AtlasLegTransforms( u1, u2, u3, u4, u5, u6 )
%ATLASLEGDH Returns DH of legs

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% DH parameters
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% general coordinates - joint angles
u = [ u1 ;      % hip yaw
      u2 ;      % hip roll
      u3 ;      % hip pitch
      u4 ;      % knee pitch
      u5 ;      % ankle pitch
      u6 ];     % ankle roll

l = [ 1 ;      % hip pitch z
      1 ;      % hip pitch x
      3 ;      % hip - knee
      4 ];     % knee - foot

% frame 0 - atlas
DH(1) = struct('a', 0, 'alpha', 0, 'theta', 0, 'd', 0);
% frame 1 - hip yaw
DH(2) = struct('a', 0, 'alpha', pi/2, 'theta', pi/2 + u(1), 'd', 0);
% frame 2 - hip roll
DH(3) = struct('a', l(1), 'alpha', -pi/2, 'theta', -pi/2 + u(2), 'd', l(2));
% frame 3 - hip pitch
DH(4) = struct('a', l(3), 'alpha', 0, 'theta', u(3), 'd', 0);
% frame 4 - knee pitch
DH(5) = struct('a', l(4), 'alpha', 0, 'theta', u(4), 'd', 0);
% frame 5 - ankle pitch
DH(6) = struct('a', 0, 'alpha', pi/2, 'theta', u(5), 'd', 0);
% frame 6 - ankle roll
DH(7) = struct('a', 0, 'alpha', 0, 'theta', u(6), 'd', 0);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Transforms
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

T01 = DH2Transform(DH(1), DH(2));
T12 = DH2Transform(DH(2), DH(3));
T23 = DH2Transform(DH(3), DH(4));
T34 = DH2Transform(DH(4), DH(5));
T45 = DH2Transform(DH(5), DH(6));
T56 = DH2Transform(DH(6), DH(7));

T = { T01, T12, T23, T34, T45, T56 };

end

