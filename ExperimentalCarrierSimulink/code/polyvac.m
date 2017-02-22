function D = polyvac(V,x)
%POLYnomial VAlues of vectors corresponding to Columns 
%POLYVAC = generalization of POLYFIT,
% for matrices V(ORD+1,512) acceleration abou 7 to 12 times
%compared to POLYVAL
% 
% Call:
%               V = polyfic(V,x)
%
%Input:     x = vector (independent variable)
%           V = columns of polynomial coefficients, usually produced by POLYFIC
%Output
%           D = data, matrix length(x)*size(V,2)
%           see also POLYFIC
%Vassili Pastushenko Aug. 2 2004

x=x(:);
%Vandermond
W=ones(size(x));
LEV=size(V,1);
for i=2:LEV
    W(:,i)=W(:,i-1).*x;
end
W=fliplr(W);
D=W*V;
