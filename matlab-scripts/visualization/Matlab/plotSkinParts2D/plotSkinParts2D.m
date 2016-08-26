% plotting is based on plotTaxelsPOSUNUTE, but copied here, so I can label
% and save figs

clc;clear;

SAVE_FIGURES = true; 

%%load data
load('./taxel_pos_zdenek/taxPosTorso.mat');
Mtorso = M;
M = [];

load('./taxel_pos_zdenek/rUpperarmTaxPos.mat');
MupperArmR = M;
M = [];

load('./taxel_pos_zdenek/rForearmTaxPos.mat');
MforearmR = rForearmTaxPos;
M = []; rForearmTaxPos = [];


%% plot 
f1 = figure(1);
clf(f1);
title('Taxel IDs - torso like skinGui');
%plotTaxelsPOSUNUTE(Mtorso,'capt');
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

f2 = figure(2);
clf(f2);
title('Taxel IDs - right upper arm like skinGui');
%plotTaxelsPOSUNUTE(MupperArmR,'capt');
axis equal;
hold on;
for i=1:1:size(MupperArmR,2)
    if not(MupperArmR(3,i)==0)
        plot(MupperArmR(1,i),MupperArmR(2,i),'oy');
    end
end
for i=1:1:size(MupperArmR,2)
    if not(MupperArmR(3,i)==0)
        text(MupperArmR(1,i),MupperArmR(2,i),int2str(i-1),'fontSize',10,'fontWeight','bold')
    end
end


f3 = figure(3);
clf(f3);
title('Taxel IDs - right forearm like skinGui');
%plotTaxelsPOSUNUTE(MforearmR,'capt');
axis equal;
hold on;
for i=1:1:size(MforearmR,2)
    if not(MforearmR(3,i)==0)
        plot(MforearmR(1,i),MforearmR(2,i),'oy');
    end
end
for i=1:1:size(MforearmR,2)
    if not(MforearmR(3,i)==0)
        text(MforearmR(1,i),MforearmR(2,i),int2str(i-1),'fontSize',10,'fontWeight','bold')
    end
end



%% saving

if SAVE_FIGURES
  saveas(f1,'TaxelIDs_torso_likeSkinGui.fig');
  print -f1 -djpeg 'TaxelIDs_torso_likeSkinGui.jpg';
  saveas(f2,'TaxelIDs_rightUpperArm_likeSkinGui.fig');
  print -f2 -djpeg 'TaxelIDs_rightUpperArm_likeSkinGui.jpg';
  saveas(f3,'TaxelIDs_rightForearm_likeSkinGui.fig');
  print -f3 -djpeg 'TaxelIDs_rightForearm_likeSkinGui.jpg';
end


