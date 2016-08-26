

generovaniPosTaxTorso;

Mtorso = M;

f1 = figure(1);
clf(f1);
title('Taxel IDs - torso like skinGui');
axis equal;
hold on;
for i=1:1:size(Mtorso,2)
    if not(Mtorso(3,i)==0)
        plot(Mtorso(1,i),Mtorso(2,i),'oy');
    end
end
for i=1:1:size(Mtorso,2)
    if not(Mtorso(3,i)==0)
        text(Mtorso(1,i),Mtorso(2,i),int2str(i-1),'fontSize',10,'fontWeight','bold')
    end
end

