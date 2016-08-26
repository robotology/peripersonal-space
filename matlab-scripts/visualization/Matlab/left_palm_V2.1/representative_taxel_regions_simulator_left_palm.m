%Edited by Matej Hoffmann
% Genova Dec 2013

clear; clc;

SAVE_FIGURES = true;
% N.B. this code is the basis for the implementation in iCub simulator in
% iCub_Sim.cpp, OdeSdlSimulation::mapPositionIntoTaxelList(...)
EXTRA_MARGIN_mm = 30;
MORE_EXTRA_MARGIN_mm = 10;

%% load stuff
%load('taxel_positions_left_palm_Marco');
load('taxel_positions_with_IDs_left_palm_Marco');
% this loads the TAXEL_IDS_AND_POSITIONS variable which comes after preprocessing of the excel sheet from Marco Maggiali - MMSP_L.xlsx
% Only columns 1, 4 and  5  were taken, copied to
% MMSP_L_positions_sorted_by_taxel_number.xlsx and SORTED by taxel number
% column, The manually copied to a matlab variable TAXEL_IDS_AND_POSITIONS
% and saved.
% first column: taxel ID - this is only for the palm - for the skin ports, this is actually 8*12 + this ID, that is the first palm taxel should be 97th value on the port, with ID of 96 - see  
% /home/matej/programming/iCub/main/app/skinGui/conf/skinGui/left_hand_V2_1.ini
% 2nd column: "x" positions of taxels in mm
% 3rd column: "y" positions of taxels in mm
% these are in FoR_9 - the "second" wrist joint - if we swap the axes x and
% y, an, in x,y,z. if we add -16 to z - there is a shift in the z-axis from
% wrist to palm

NR_TAXELS = size(TAXEL_IDS_AND_POSITIONS,1);
TAXEL_ID_OFFSET_PALM_TO_HAND = 96;


%% FoR transformations

%1) add empty z column
taxel_positions_FoR_9 = [];
for i=1:NR_TAXELS;
    taxel_positions_FoR_9(i,1) = TAXEL_IDS_AND_POSITIONS(i,3); % swap x and y
    taxel_positions_FoR_9(i,2) = TAXEL_IDS_AND_POSITIONS(i,2);
    taxel_positions_FoR_9(i,3) = -16; % w.r.t wrist joint position, the palm skin is 16 mm more "up", which is -16 w.r.t FoR9 z-axis
end

% 2) transform to FoR_10 (palm)
% I ran /media/DATA/my_matlab/icub_kinematics/ICubFwdKinNew/WaistLeftArmFwdKin/WaistLeftArmFwdKin_asScript.m
% to get the rototrans. matrix from FoR9 (second wrist joint) to FoR10
% (palm) --  G_910 in workspace (for zero joint angles theta)
% 

G_910 = [1.0000         0         0   62.5000;
         0    1.0000         0         0;
         0         0    1.0000  -16.0000;
         0         0         0    1.0000];
% this is in fact just a translation - so I could also do it without the whole rototranslation matrix, but let's keep it     
G_910_inv = inv(G_910); % to remap the coordinates, we need to use the inverse matrix       

taxel_positions_FoR_10=[];
taxel_positions_FoR_10_doublecheck=[];
for j=1:NR_TAXELS
    
   row_vector_9 = []; row_vector_10 = [];
   column_vector_homo_9=[]; column_vector_homo_10=[];
    
   row_vector_9 = taxel_positions_FoR_9(j,:);
   column_vector_homo_9 = row_vector_9';
   column_vector_homo_9(4)=1;
   column_vector_homo_10 = G_910_inv * column_vector_homo_9;
   column_vector_homo_10(4)=[];
   row_vector_10 = column_vector_homo_10';
   taxel_positions_FoR_10(j,:)=row_vector_10;
   
   
   %for doublechecking we do the translations manually 
   taxel_positions_FoR_10_doublecheck(j,1) = taxel_positions_FoR_9(j,1)-62.5;
   taxel_positions_FoR_10_doublecheck(j,2) = taxel_positions_FoR_9(j,2);
   taxel_positions_FoR_10_doublecheck(j,3) = taxel_positions_FoR_9(j,3)+16;
   
   if (~ isequal(taxel_positions_FoR_10,taxel_positions_FoR_10_doublecheck))
       disp('Transformed arrays are not equal!');
   end
  
   
end
       
%% visualize   

f2 = figure(2);
clf(f2);
length = 10; % for ref. frame
title('Taxel positions left palm FoR (nr. 10 in arm kinematics)');
hold on;
% we swap the axes for visualiation to match better with the palm
plot(taxel_positions_FoR_10(:,2),taxel_positions_FoR_10(:,1),'xb');
for i=1:NR_TAXELS
   if ( ((i-1+TAXEL_ID_OFFSET_PALM_TO_HAND) == 107) || ((i-1+TAXEL_ID_OFFSET_PALM_TO_HAND) == 119) || ((i-1+TAXEL_ID_OFFSET_PALM_TO_HAND) == 131) || ((i-1+TAXEL_ID_OFFSET_PALM_TO_HAND) == 139) ) %thermal pads
         % text(taxel_positions_FoR_10(i,2),taxel_positions_FoR_10(i,1),int2str(i-1+TAXEL_ID_OFFSET_PALM_TO_HAND),'FontSize',8); 
   else
            text(taxel_positions_FoR_10(i,2),taxel_positions_FoR_10(i,1),int2str(i-1+TAXEL_ID_OFFSET_PALM_TO_HAND),'FontSize',14); 
   end 
end
h = quiver(0 ,0, 10,0);
set(h, 'Color', 'g', 'LineWidth', 2, 'MaxHeadSize', 4, 'ShowArrowHead', 'on');
text(5,0,'y');
h2 = quiver(0,0, 0,10);
set(h2, 'Color', 'b', 'LineWidth', 2, 'MaxHeadSize', 4, 'ShowArrowHead', 'on')
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
xlabel('Taxel position y (mm)');
ylabel('Taxel position x (mm)');
axis equal;
grid on;
hold off;

%% save figures

if SAVE_FIGURES
    saveas(f2,'Taxel_positions_left_palm_FoR10_withSkinEmulationForSIMregions.fig');
    print -f2 -djpeg 'Taxel_positions_left_palm_FoR10__withSkinEmulationForSIMregions.jpg';
end

