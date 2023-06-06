function [DCM_BA] = Quat2DCM(Quaternion_BA)

% OROLAB 2023
% (c) Dr. Marcello Romano: written on 2019-04-11
%
% INPUTS
% quaternion_BA = [4,1 matrix of real numbers satifying constraint norm(Quaternion_BA)=1]
% WARNING: the scalar elements is the fourth element!
% 
% OUTPUTS
% DCM_BA = [3,3 matrix in SO(3)]= Direction Cosine Matrix from CCS A to
% CCS B = Rotation Matrix from CCS B to CCS A

normaccuracy=10e-07;

if isnumeric(Quaternion_BA)
% CHECK INPUT
if size(Quaternion_BA,1) ~=4 || size(Quaternion_BA,2) ~=1
    error('Input Quaternion_BA has to be a (4,1) matrix')
end

if norm(1-norm(Quaternion_BA))>normaccuracy
    error('Input Quaternion_BA has to have unit norm')
end
% End CHECK INPUT
end
qvect = Quaternion_BA(1:3,1);
qscalar = Quaternion_BA(4,1);

DCM_BA=(qscalar^2-qvect'*qvect)*eye(3)+2*(qvect*qvect')-2*qscalar*VectProdMatrix(qvect);