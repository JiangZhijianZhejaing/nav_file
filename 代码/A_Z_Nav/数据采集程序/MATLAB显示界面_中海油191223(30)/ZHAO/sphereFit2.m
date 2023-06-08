function [Center,Radius] = sphereFit2(sData)
% this fits a sphere to a collection of data using a closed form for the
% solution (opposed to using an array the size of the data set). 
% Minimizes Sum((x-xc)^2+(y-yc)^2+(z-zc)^2-r^2)^2
% x,y,z are the data, xc,yc,zc are the sphere's center, and r is the radius

% Assumes that points are not in a singular configuration, real numbers, ...
% if you have coplanar data, use a circle fit with svd for determining the
% plane, recommended Circle Fit (Pratt method), by Nikolai Chernov
% http://www.mathworks.com/matlabcentral/fileexchange/22643

% Input:
% sData: n x 3 matrix of cartesian data
% Outputs:
% Center: Center of sphere 
% Radius: Radius of sphere
% Author:
% ZHIZUWEI, USTC&IIM
X = sData(:,1);
Y = sData(:,2);
Z = sData(:,3);
E = ones(size(sData,1),1);
H = [X.^2, Y.^2, Z.^2, X.*Y ,X.*Z, Y.*Z, X, Y, Z];
K = (H'*H)\H'*E;
M = [ K(1) K(4)/2 K(5)/2
    K(4)/2 K(2) K(6)/2
    K(5)/2 K(6)/2 K(3) ];
Center = -0.5*[K(7) K(8) K(9)]/M;
Radius=sqrt(mean(sum([X-Center(1),Y-Center(2),Z-Center(3)].^2,2)));
