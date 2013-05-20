function [ T0f ] = AtlasLegFK( u1, u2, u3, u4, u5, u6 )
%ATLASLEGFK Foot transform given joint angles

T = AtlasLegTransforms(u1, u2, u3, u4, u5, u6);
T0f = T{1}*T{2}*T{3}*T{4}*T{5}*T{6};

end

