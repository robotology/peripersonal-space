% Matej Hoffmann, 12.5.2015, adapted from the script for left forearm which
% is much more comprehensive

clear all;

SAVE_FIGURES = false;
%% Init taxel positions from Andrea's calibration

load right_forearm_taxel_pos_mesh.mat; % the no_mesh is also possible, but there are no normals, so you can't overlay the triangular modules
taxel_pos = right_forearm_taxel_pos_mesh; 
[M,N] = size(taxel_pos);


%% Plot positions of calibrated taxels - Andrea

f1 = figure(1);
clf(f1);
title('Positions of taxels with their IDs (in 1st wrist FoR - FoR_8)');
hold on;

for i=1:M
    if (nnz(taxel_pos(i,:)) > 1) % it's not an all-zero row
       plot3(taxel_pos(i,1),taxel_pos(i,2),taxel_pos(i,3),'xb');
       text(taxel_pos(i,1),taxel_pos(i,2),taxel_pos(i,3),int2str(i-1)); % taxel ID = row nr. -1
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
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Martin Varga

load m_m.mat;
load n_n.mat;


%change acording to new axes
for i= 1:3
    m_m(i,2) = - m_m(i,2);
    m_m(i,3) = - m_m(i,3);
    m_m(i,5) = - m_m(i,5);
    m_m(i,6) = - m_m(i,6);
end    

for i= 1:8
    n_n(i,2) = - n_n(i,2);
    n_n(i,3) = - n_n(i,3);
    n_n(i,5) = - n_n(i,5);
    n_n(i,6) = - n_n(i,6);
end    
 % draws big "rectangles"
for i= 1:8
      
    plot3([n_n(i,1) n_n(i,4)],[n_n(i,5) n_n(i,5)],[n_n(i,3) n_n(i,3)], 'Color', 'r');
    plot3([n_n(i,4) n_n(i,4)],[n_n(i,5) n_n(i,5)],[n_n(i,3) n_n(i,6)], 'Color', 'r');
    plot3([n_n(i,4) n_n(i,1)],[n_n(i,5) n_n(i,5)],[n_n(i,6) n_n(i,6)], 'Color', 'r');
    plot3([n_n(i,1) n_n(i,1)],[n_n(i,5) n_n(i,5)],[n_n(i,3) n_n(i,6)], 'Color', 'r');
    
    plot3([n_n(i,1) n_n(i,4)],[n_n(i,2) n_n(i,2)],[n_n(i,3) n_n(i,3)], 'Color', 'r');
    plot3([n_n(i,4) n_n(i,4)],[n_n(i,2) n_n(i,2)],[n_n(i,3) n_n(i,6)], 'Color', 'r');
    plot3([n_n(i,4) n_n(i,1)],[n_n(i,2) n_n(i,2)],[n_n(i,6) n_n(i,6)], 'Color', 'r');
    plot3([n_n(i,1) n_n(i,1)],[n_n(i,2) n_n(i,2)],[n_n(i,3) n_n(i,6)], 'Color', 'r');
    
    plot3([n_n(i,1) n_n(i,1)],[n_n(i,2) n_n(i,5)],[n_n(i,3) n_n(i,3)], 'Color', 'r');
    plot3([n_n(i,4) n_n(i,4)],[n_n(i,2) n_n(i,5)],[n_n(i,3) n_n(i,3)], 'Color', 'r');
    plot3([n_n(i,4) n_n(i,4)],[n_n(i,2) n_n(i,5)],[n_n(i,6) n_n(i,6)], 'Color', 'r');
    plot3([n_n(i,1) n_n(i,1)],[n_n(i,2) n_n(i,5)],[n_n(i,6) n_n(i,6)], 'Color', 'r');
     
end

for i= 1:3
    %uncoment to see line conecting minimal and maximal point of rectangle 
    %plot3([min_max(i,1) min_max(i,4)],[min_max(i,2) min_max(i,5)],[min_max(i,3) min_max(i,6)]);
    
    plot3([m_m(i,1) m_m(i,4)],[m_m(i,5) m_m(i,5)],[m_m(i,3) m_m(i,3)], 'Color', 'g');
    plot3([m_m(i,4) m_m(i,4)],[m_m(i,5) m_m(i,5)],[m_m(i,3) m_m(i,6)], 'Color', 'g');
    plot3([m_m(i,4) m_m(i,1)],[m_m(i,5) m_m(i,5)],[m_m(i,6) m_m(i,6)], 'Color', 'g');
    plot3([m_m(i,1) m_m(i,1)],[m_m(i,5) m_m(i,5)],[m_m(i,3) m_m(i,6)], 'Color', 'g');
    
    plot3([m_m(i,1) m_m(i,4)],[m_m(i,2) m_m(i,2)],[m_m(i,3) m_m(i,3)], 'Color', 'g');
    plot3([m_m(i,4) m_m(i,4)],[m_m(i,2) m_m(i,2)],[m_m(i,3) m_m(i,6)], 'Color', 'g');
    plot3([m_m(i,4) m_m(i,1)],[m_m(i,2) m_m(i,2)],[m_m(i,6) m_m(i,6)], 'Color', 'g');
    plot3([m_m(i,1) m_m(i,1)],[m_m(i,2) m_m(i,2)],[m_m(i,3) m_m(i,6)], 'Color', 'g');
    
    plot3([m_m(i,1) m_m(i,1)],[m_m(i,2) m_m(i,5)],[m_m(i,3) m_m(i,3)], 'Color', 'g');
    plot3([m_m(i,4) m_m(i,4)],[m_m(i,2) m_m(i,5)],[m_m(i,3) m_m(i,3)], 'Color', 'g');
    plot3([m_m(i,4) m_m(i,4)],[m_m(i,2) m_m(i,5)],[m_m(i,6) m_m(i,6)], 'Color', 'g');
    plot3([m_m(i,1) m_m(i,1)],[m_m(i,2) m_m(i,5)],[m_m(i,6) m_m(i,6)], 'Color', 'g');
    
    
    
end

i = 3;
    plot3([n_n(i,1) n_n(i,4)],[n_n(i,5) n_n(i,5)],[n_n(i,3) n_n(i,3)], 'Color', 'b');
    plot3([n_n(i,4) n_n(i,4)],[n_n(i,5) n_n(i,5)],[n_n(i,3) n_n(i,6)], 'Color', 'b');
    plot3([n_n(i,4) n_n(i,1)],[n_n(i,5) n_n(i,5)],[n_n(i,6) n_n(i,6)], 'Color', 'b');
    plot3([n_n(i,1) n_n(i,1)],[n_n(i,5) n_n(i,5)],[n_n(i,3) n_n(i,6)], 'Color', 'b');
    
    plot3([n_n(i,1) n_n(i,4)],[n_n(i,2) n_n(i,2)],[n_n(i,3) n_n(i,3)], 'Color', 'b');
    plot3([n_n(i,4) n_n(i,4)],[n_n(i,2) n_n(i,2)],[n_n(i,3) n_n(i,6)], 'Color', 'b');
    plot3([n_n(i,4) n_n(i,1)],[n_n(i,2) n_n(i,2)],[n_n(i,6) n_n(i,6)], 'Color', 'b');
    plot3([n_n(i,1) n_n(i,1)],[n_n(i,2) n_n(i,2)],[n_n(i,3) n_n(i,6)], 'Color', 'b');
    
    plot3([n_n(i,1) n_n(i,1)],[n_n(i,2) n_n(i,5)],[n_n(i,3) n_n(i,3)], 'Color', 'b');
    plot3([n_n(i,4) n_n(i,4)],[n_n(i,2) n_n(i,5)],[n_n(i,3) n_n(i,3)], 'Color', 'b');
    plot3([n_n(i,4) n_n(i,4)],[n_n(i,2) n_n(i,5)],[n_n(i,6) n_n(i,6)], 'Color', 'b');
    plot3([n_n(i,1) n_n(i,1)],[n_n(i,2) n_n(i,5)],[n_n(i,6) n_n(i,6)], 'Color', 'b');
%end Martin VArga
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%ylim([-45 15]);
%xlim([-30 30]);
xlabel('Taxel position x (m)');
set(gca,'XDir','reverse');
ylabel('Taxel position y (m)');
set(gca,'YDir','reverse');
zlabel('Taxel position z (m)');
%set(gca,'ZDir','reverse');
axis equal;
hold off;





%% save figures
if SAVE_FIGURES
    saveas(f1,'Taxel_positions_left_forearm.fig');
    print -f1 -djpeg 'Taxel_positions_left_forearm.jpg';
   
end