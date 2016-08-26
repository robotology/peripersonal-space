%script create matrix M with positions of taxels
%not same like in iCub visualization tool- missing some tranformation like
%rotation, translation..
%see forearm_ini_generator.xlsx
function M=createTaxelPos(Trians,Taxs)
M=[];
for i=1:1:size(Trians,2)
    alpha=Trians(3,i);
    for j=1:1:size(Taxs,2)
    X=Trians(1:2,i)+[cos(alpha) -sin(alpha);sin(alpha) cos(alpha)]*Taxs(1:2,j);    
    M=[M,[X;Trians(4,i).*Taxs(3,j)]];
    end

end