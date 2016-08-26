%Edited by Matej Hoffmann
% Genova Dec 2013

clear; clc;
SAVE_FIGURES = true;
% N.B. this code is the basis for the implementation in iCub simulator in
% iCub_Sim.cpp, OdeSdlSimulation::mapPositionIntoTaxelList(...)
EXTRA_MARGIN_mm = 30;
MORE_EXTRA_MARGIN_mm = 10;

%% load stuff
load('taxel_positions_with_IDs_right_palm_Marco');
% this loads the TAXEL_IDS_AND_POSITIONS variable which comes after preprocessing of the excel sheet from Marco Maggiali - MMSP_R.xlsx
% Only columns 1, 4 and  5  were taken, copied to
% MMSP_R_positions_sorted_by_taxel_number.xlsx and SORTED by taxel number
% column. The manually copied to a matlab variable TAXEL_IDS_AND_POSITIONS
% and saved
% first column: taxel ID - this is only for the palm - for the skin ports, this is actually 8*12 + this ID, that is the first palm taxel should be 97th value on the port, with ID of 96 - see  
% /home/matej/programming/iCub/main/app/skinGui/conf/skinGui/left_hand_V2_1.ini
% 2nd column: "x" positions of taxels in mm
% 3rd column: "y" positions of taxels in mm
% Note the different orientation of axes in right hand w.r.t left hand!
% these are position-wise in wrist joint and with "x" pointing to thumb and "y" positive toward fingers; therefore we swap the x and y and multiply the new y by -1 
%and we have x and y axis of FoR right palm; then we just do the translation from wrist to palm: -62.5 in x; we set z-coordinate directly to 0 in FoR_10

NR_TAXELS = size(TAXEL_IDS_AND_POSITIONS,1);
TAXEL_ID_OFFSET_PALM_TO_HAND = 96;


%% FoR transformations

taxel_positions_FoR_10 = [];
for i=1:NR_TAXELS;
    taxel_positions_FoR_10(i,1) = TAXEL_IDS_AND_POSITIONS(i,3) -62.5; % swap x and y and shift the new x
    taxel_positions_FoR_10(i,2) = -1* TAXEL_IDS_AND_POSITIONS(i,2); % put x to y and change orientation
    taxel_positions_FoR_10(i,3) = 0;
end
       
%% visualize   


f3 = figure(3);
clf(f3);
length = 10; % for ref. frame
title('Taxel positions right palm FoR (nr. 10 in arm kinematics)');
hold on;
% we swap the axes for visualiation to match better with the palm
plot(taxel_positions_FoR_10(:,2),taxel_positions_FoR_10(:,1),'xb');

for i=1:NR_TAXELS
   if ( ((i-1+TAXEL_ID_OFFSET_PALM_TO_HAND) == 107) || ((i-1+TAXEL_ID_OFFSET_PALM_TO_HAND) == 119) || ((i-1+TAXEL_ID_OFFSET_PALM_TO_HAND) == 131) || ((i-1+TAXEL_ID_OFFSET_PALM_TO_HAND) == 139) ) %thermal pads
          %text(taxel_positions_FoR_10(i,2),taxel_positions_FoR_10(i,1),int2str(i-1+TAXEL_ID_OFFSET_PALM_TO_HAND),'FontSize',8); 
   else
            text(taxel_positions_FoR_10(i,2),taxel_positions_FoR_10(i,1),int2str(i-1+TAXEL_ID_OFFSET_PALM_TO_HAND),'FontSize',14); 
   end 
end
h = quiver(0 ,0, 10,0);
set(h, 'Color', 'g', 'LineWidth', 2, 'MaxHeadSize', 4, 'ShowArrowHead', 'on');
text(5,0,'y');
h2 = quiver(0,0, 0,10);
set(h2, 'Color', 'r', 'LineWidth', 2, 'MaxHeadSize', 4, 'ShowArrowHead', 'on')
text(0,5,'x');

%  line([x0 x1],[y0 y1],'Color','r','LineStyle','--','Marker','x'); 
% vertical lines
line([-26 -26],[-12 3],'Color','k','LineStyle','--','Marker','x'); 
line([-5.5 -5.5],[-12 3],'Color','k','LineStyle','--','Marker','x'); 
line([10 10],[-14 3],'Color','k','LineStyle','--','Marker','x'); 
line([0 0],[-14 -40],'Color','k','LineStyle','--','Marker','x'); 
line([30 30],[-40 3],'Color','k','LineStyle','--','Marker','x'); 
% horizontal lines
line([-26 30],[3 3],'Color','k','LineStyle','--','Marker','x');
line([-26 10],[-12 -12],'Color','k','LineStyle','--','Marker','x');
line([0 30],[-14 -14],'Color','k','LineStyle','--','Marker','x'); 
line([0 30],[-24 -24],'Color','k','LineStyle','--','Marker','x'); 
line([0 30],[-40 -40],'Color','k','LineStyle','--','Marker','x');


if EXTRA_MARGIN_mm
    % vertical lines
    line([-26-EXTRA_MARGIN_mm-1.5*MORE_EXTRA_MARGIN_mm  -26-EXTRA_MARGIN_mm-1.5*MORE_EXTRA_MARGIN_mm],[-12-2 3+MORE_EXTRA_MARGIN_mm],'Color','r','LineStyle','--','Marker','x'); 
    line([-5.5 -5.5],[-12-2 3+MORE_EXTRA_MARGIN_mm],'Color','r','LineStyle','--','Marker','x'); 
    line([10 10],[-14 3+MORE_EXTRA_MARGIN_mm],'Color','r','LineStyle','--','Marker','x'); 
    line([0-EXTRA_MARGIN_mm-2*MORE_EXTRA_MARGIN_mm 0-EXTRA_MARGIN_mm-2*MORE_EXTRA_MARGIN_mm],[-14 -40-EXTRA_MARGIN_mm-2*MORE_EXTRA_MARGIN_mm],'Color','r','LineStyle','--','Marker','x'); 
    line([30+EXTRA_MARGIN_mm 30+EXTRA_MARGIN_mm],[-40-EXTRA_MARGIN_mm-2*MORE_EXTRA_MARGIN_mm 3+MORE_EXTRA_MARGIN_mm],'Color','r','LineStyle','--','Marker','x'); 
    % horizontal lines
    line([-26-EXTRA_MARGIN_mm-1.5*MORE_EXTRA_MARGIN_mm 30+EXTRA_MARGIN_mm],[3+MORE_EXTRA_MARGIN_mm 3+MORE_EXTRA_MARGIN_mm],'Color','r','LineStyle','--','Marker','x');
    line([-26-EXTRA_MARGIN_mm-1.5*MORE_EXTRA_MARGIN_mm 10],[-12-2 -12-2],'Color','r','LineStyle','--','Marker','x');
    line([0-EXTRA_MARGIN_mm 30+EXTRA_MARGIN_mm],[-14 -14],'Color','r','LineStyle','--','Marker','x'); 
    line([0-EXTRA_MARGIN_mm-2*MORE_EXTRA_MARGIN_mm 30+EXTRA_MARGIN_mm],[-24 -24],'Color','r','LineStyle','--','Marker','x'); 
    line([0-EXTRA_MARGIN_mm-2*MORE_EXTRA_MARGIN_mm 30+EXTRA_MARGIN_mm],[-40-EXTRA_MARGIN_mm-2*MORE_EXTRA_MARGIN_mm -40-EXTRA_MARGIN_mm-2*MORE_EXTRA_MARGIN_mm],'Color','r','LineStyle','--','Marker','x');
    
    ylim([-42-EXTRA_MARGIN_mm-2*MORE_EXTRA_MARGIN_mm 10+MORE_EXTRA_MARGIN_mm]);
    xlim([-30-EXTRA_MARGIN_mm 30+EXTRA_MARGIN_mm]);
else
    ylim([-42 10]);
    xlim([-30 30]);
end

%ylim([-45 15]);
%xlim([-30 30]);
xlabel('Taxel position y (mm)');
set(gca,'XDir','reverse');
ylabel('Taxel position x (mm)');
axis equal;
grid on;
hold off;

if SAVE_FIGURES
    saveas(f3,'Taxel_positions_right_palm_FoR10_withSkinEmulationForSIMregions.fig');
    print -f3 -djpeg 'Taxel_positions_right_palm_FoR10_withSkinEmulationForSIMregions.jpg';
end


