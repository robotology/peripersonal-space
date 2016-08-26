%% About 
% Script to load taxel position for visualize the PPS of the right forearm of iCub
% Author: NGUYEN Dong Hai Phuong
% Email: phuong.nguyen@iit.it; ph17dn@gmail.com

%%
SAVE_FIGURES = false;
%% Init taxel positions from Andrea's calibration

load right_forearm_taxel_pos_mesh.mat; % the no_mesh is also possible, but there are no normals, so you can't overlay the triangular modules
taxel_pos = right_forearm_taxel_pos_mesh; 
[M,N] = size(taxel_pos);


% Plot positions of calibrated taxels - Andrea

f41 = figure(41);
clf(f41);
title('Positions of taxels with their IDs (in 1st wrist FoR - FoR_8)');
hold on;

for i=1:M
    if (nnz(taxel_pos(i,:)) > 1) % it's not an all-zero row
       plot3(taxel_pos(i,1),taxel_pos(i,2),taxel_pos(i,3),'xb');
%        text(taxel_pos(i,1),taxel_pos(i,2),taxel_pos(i,3),int2str(i-1)); % taxel ID = row nr. -1
        if (mod(i-3,12)==0)
            text(taxel_pos(i,1),taxel_pos(i,2),taxel_pos(i,3),int2str(i-1),'Color','r'); 
        end
    end
end
 h = quiver3(0 ,0, 0,0.02,0,0);
 set(h, 'Color', 'r', 'LineWidth', 2, 'MaxHeadSize', 4, 'ShowArrowHead', 'on');
 text(0.01,0,0,'x');
 h2 = quiver3(0,0,0, 0,0.02,0);
 set(h2, 'Color', 'g', 'LineWidth', 2, 'MaxHeadSize', 4, 'ShowArrowHead', 'on')
 text(0,0.01,0,'y');
 h3 = quiver3(0,0,0, 0,0,0.02);
 set(h3, 'Color', 'b', 'LineWidth', 2, 'MaxHeadSize', 4, 'ShowArrowHead', 'on')
 text(0,0,0.01,'z');
 grid on;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%