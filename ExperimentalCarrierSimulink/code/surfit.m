function BAS=surfit(D,ORDVER,ORDHOR)
%	polynomial SURface FIT with separate orders
%   in vertical and horizontal directions
%Input:		
%		D = matrix
%		ORDVER polynomial ORDer for VERtical fit, def. 1
%       ORDHOR polynomial ORDer for HORizontal fit, def. ORDVER
%Output:	
%       BAS = Fitted Surface
%--------------------------------
% Demo
%       [X,Y]=meshgrid(1:100,1:100);
%       D=(X.^3/100-X.*Y+3000-Y.^2/2)/1000;
%       F=surfit(D,2,3);
%       norm(D(:)-F(:),Inf)
% ----------------------------------
%See also: POLYFIC, POLYVAC
%    Vassili Pastushenko,	11.11.2004. [PFE]
%==============================================================
NAR=nargin;
if NAR<1, error('Data needed'),end
if NAR<2,ORDVER=1;end
if NAR<3,ORDHOR=ORDVER;end
[R,C]=size(D);
x=(0:C-1)/(C-1); 
y=(0:R-1)/(R-1);
BAS=polyfic(y,D,ORDVER);
BV=polyvac(BAS,y)';
V=polyfic(x,BV,ORDHOR);
BAS=polyvac(V,x)';