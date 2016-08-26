function f=plotTaxels(M,numTax)
%M-matrix, first row-positions x, second row positions y, third row
%visible=1 invisible=0
%optional parameter--numTax-if numTax=='capt' then plot caption with numbers of taxels
f=figure;
axis equal;
hold on;
for i=1:1:size(M,2)
if not(M(3,i)==0)
plot(M(1,i),M(2,i),'oy');
end
end

if(nargin>1)
if(numTax=='capt')
for i=1:1:size(M,2)
if not(M(3,i)==0)
text(M(1,i),M(2,i),int2str(i-1),'fontSize',10,'fontWeight','bold')
end
end
end
end