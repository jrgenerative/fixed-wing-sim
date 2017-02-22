function V = polyfic(x,y,ORD)
%   Matlab function POLYFIT(x,y,ORD) allows only the same size of
%   x and y inputs. If x is a vector of N elements, and 
%   size(y) =[N, M], and one needs to fit each column of y with a polynomial,
%   POLYFIT can only be used in connection with some loop FOR, WHILE etc.
%   This essentially decreases the practical power of MATLAB, and can become quite
%   nasty, especially in the case of big matrices (image analysis: tyoically 512*512).
%   Program POLYFIC allows to fit colomns, using the power of matrix operations, which
%   leads to acceleration 20 to 80 times (512*512, ORDers 1:15)
% 
%POLYFIC = POLYnomial FIT of Columns: generalization of POLYFIT 
% Fits polynomials of degree ORD to columns of data.
%
% Call:
%               V = polyfic(x,y,ORD)
%
%Input:     x = vector of independent variable
%           y = data
%           ORD = polynomial order (ORD=3 for cubic)
%Output
%           V = polynomials in columns, matrix ORD+1 * nColumns(y)
%           See also: POLYVAC
%Vassili Pastushenko Aug. 2 2004

x=x(:); %Make column of x;
[R,C]=size(y);

LX=length(x);

%Vandermond
V=ones(size(x));
for i=2:2*ORD+1
    V(:,i)=V(:,i-1).*x;
end
V=fliplr(V);
VV=sum(V);
for i=1:ORD+1
    M(i,:)=VV(i:i+ORD);
end
R=V(:,ORD+1:2*ORD+1);
B=(y.'*R)';
V=M\B;
