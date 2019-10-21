function [Kx,Kn] = kronVectorization(Ax,Bx,An,Bn,Kdes,CONFIG)
%KRONVECTORIZATION is a gains optimization for the iCub linearized joint 
%                  space dynamics through vectorization and Kronecher product.
%   KRONVECTORIZATION solves the least square problem: x = pinv(M)*xdes 
%   where x is a vectorization of the feedback control gains, xdes is the 
%   vectorization of the optimization objective and M is a proper matrix.
%
%   KRONVECTORIZATION also takes into account the following constraints on
%   the gains matrices: Kx should be block diagonal; Kx(1:3,1:3) should be 
%   diagonal; Kx(4:6,4:6) should be symmetric. Kn should be symmetric. The
%   positive definiteness constraint is only verified a-posteriori. in 
%   particular, the matrix Kn is enforced to be positive definite with the 
%   addition of a regulation term.
%    
%   [Kx,Kn] = KRONVECTORIZATION(Ax,Bx,An,Bn,Kdes,config) takes as inputs 
%   the pre and post multiplier of the gains matrices in the linearized 
%   system dynamics, i.e. AX,BX,AN,BN. KDES is the objective matrix. CONFIG 
%   is a structure containing all the utility parameters.
%
%   The output are the optimized gains matrices, Kx [6x6] and Kn [ndof x 
%   ndof]
%
% Author : Gabriele Nava (gabriele.nava@iit.it)
% Genova, May 2016

% ------------Initialization----------------
% setup parameters

ndof        = size(CONFIG.ndof,1);
% pinv_toll = CONFIG.pinv_tol;
pinv_damp   = CONFIG.pinv_damp;

kronMom     = kron(Bx',Ax);
kronNull    = kron(Bn',An);

indexDiag     = 1:36;
indexDiagMatr = reshape(indexDiag,[6,6]);
diagonalIndex = sort(diag(indexDiagMatr));
kronMom       = kronMom(:,diagonalIndex);

MKron       = [kronMom kronNull];
xdes        = Kdes(:);

%% First task generator
indexTotal  = 1:ndof^2;
indexMatrix = reshape(indexTotal,[ndof,ndof]);

if sum(CONFIG.feet_on_ground) == 2
 
numberofJoints = 11;

indexBlock1    = indexMatrix(1:numberofJoints,1:numberofJoints);
indexBlock2    = indexMatrix(numberofJoints+1:end,1:numberofJoints);
indexBlock3    = indexMatrix(1:numberofJoints,numberofJoints+1:end);
indexBlock4    = indexMatrix(numberofJoints+1:end,numberofJoints+1:end);

indexFirstTask = sort([indexBlock1(:);indexBlock2(:);indexBlock3(:)]);
indexNull      = sort(indexBlock4(:));

elseif CONFIG.feet_on_ground(1) == 1 && sum(CONFIG.feet_on_ground) == 1
    
numberofJoints = 11;

indexBlock1    = indexMatrix(:,1:numberofJoints);
indexBlock2    = indexMatrix(:,numberofJoints+6+1:end);
indexBlock3    = indexMatrix(numberofJoints+6+1:end,numberofJoints+1:numberofJoints+6);
indexBlock4    = indexMatrix(1:numberofJoints,numberofJoints+1:numberofJoints+6);
indexBlock5    = indexMatrix(numberofJoints+1:numberofJoints+6,numberofJoints+1:numberofJoints+6);

indexFirstTask = sort([indexBlock1(:);indexBlock2(:);indexBlock3(:);indexBlock4(:)]);
indexNull      = sort(indexBlock5(:));

elseif CONFIG.feet_on_ground(1) == 0 && sum(CONFIG.feet_on_ground) == 1

numberofJoints = 11+6;

indexBlock1    = indexMatrix(1:numberofJoints,1:numberofJoints);
indexBlock2    = indexMatrix(numberofJoints+1:end,1:numberofJoints);
indexBlock3    = indexMatrix(1:numberofJoints,numberofJoints+1:end);
indexBlock4    = indexMatrix(numberofJoints+1:end,numberofJoints+1:end);

indexFirstTask = sort([indexBlock1(:);indexBlock2(:);indexBlock3(:)]);
indexNull      = sort(indexBlock4(:));

end

xdesFirstTask          = xdes(indexFirstTask);
MKronFirstTask         = MKron(indexFirstTask,:);
pinvMKronFirstTask     = pinvDamped(MKronFirstTask,pinv_damp);
% pinvMKronFirstTask   = pinv(MKronFirstTask,pinv_toll);
NullMKron              = eye(size(MKronFirstTask,2))-pinvMKronFirstTask*MKronFirstTask;

%% Null Space generator
xdesNull           = xdes(indexNull);
MKronNull          = MKron(indexNull,:);
pinvMKronNull      = pinvDamped(MKronNull,pinv_damp);
% pinvMKronNull    = pinv(MKronNull,pinv_toll);
x0                 = pinvMKronNull*xdesNull;

%% Final vector
x                  = pinvMKronFirstTask*xdesFirstTask+ NullMKron*x0;

%% Gain matrices
vettKMom           = x(1:6);
vettKn             = x(7:end);
Kx                 = diag(vettKMom);
Kn                 = reshape(vettKn,[ndof,ndof]);

end


