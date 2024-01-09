function Plot3D_Ellipse(C,M,nsigma,color,style,resolution)
%  Autor: Unknown 
% This function plots a 3D ellipse given:
% C -> covariance matrix  (3 x 3 )
% M -> mean (center position of the ellipse)  ( 1 x 3)
% nsigma -> deviation standard number
% color
% style

[U,L] = eig(C);

% For N standard deviations spread of data, the radii of the eliipsoid will
% be given by N*SQRT(eigenvalues).

N = nsigma; % choose your own N
radii = N*sqrt(diag(L));

% generate data for "unrotated" ellipsoid
[xc,yc,zc] = ellipsoid(0,0,0,radii(1),radii(2),radii(3),resolution);

% rotate data with orientation matrix U and center M
a = kron(U(:,1),xc); b = kron(U(:,2),yc); c = kron(U(:,3),zc);
data = a+b+c; n = size(data,2);
x = data(1:n,:)+M(1); y = data(n+1:2*n,:)+M(2); z = data(2*n+1:end,:)+M(3);

% now plot the rotated ellipse
%sc = surf(x,y,z);
%sc = mesh(xc,yc,zc,'EdgeColor',color,'LineStyle',style);
 if isreal(x)&& isreal(y)&& isreal(z)  
sc = mesh(x,y,z,'EdgeColor',color,'LineStyle',style);
 end
%shading interp
%title('actual ellipsoid represented by data: C and M')
%axis equal
alpha(0.1)