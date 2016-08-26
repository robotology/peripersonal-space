%Edited by Matej Hoffmann
% Genova Dec 2013

clear; clc;
SAVE_FIGURES = false;
PRODUCE_OUTPUT_FILE = false;

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

% plot original taxel positions
f1 = figure(1);
clf(f1);

hold on;
plot(TAXEL_IDS_AND_POSITIONS(:,2),TAXEL_IDS_AND_POSITIONS(:,3),'.b');
for i=1:NR_TAXELS
   text(TAXEL_IDS_AND_POSITIONS(i,2),TAXEL_IDS_AND_POSITIONS(i,3),int2str(TAXEL_IDS_AND_POSITIONS(i,1))); 
end
xlim([-35 35]);
ylim([0 70]);
xlabel('Taxel position "x" (mm)');
ylabel('Taxel position "y" (mm)');
axis equal;
hold off;

f2 = figure(2);
clf(f2);
length = 10; % for ref. frame
title('Taxel positions right palm FoR (nr. 10 in arm kinematics)');
hold on;
% we swap the axes for visualiation to match better with the palm
plot(taxel_positions_FoR_10(:,2),taxel_positions_FoR_10(:,1),'xb');
for i=1:NR_TAXELS
   text(taxel_positions_FoR_10(i,2),taxel_positions_FoR_10(i,1),int2str(i-1+TAXEL_ID_OFFSET_PALM_TO_HAND)); 
end
h = quiver(0 ,0, 10,0);
set(h, 'Color', 'g', 'LineWidth', 2, 'MaxHeadSize', 4, 'ShowArrowHead', 'on');
text(5,0,'y');
h2 = quiver(0,0, 0,10);
set(h2, 'Color', 'r', 'LineWidth', 2, 'MaxHeadSize', 4, 'ShowArrowHead', 'on')
text(0,5,'x');

%ylim([-45 15]);
%xlim([-30 30]);
xlabel('Taxel position y (mm)');
set(gca,'XDir','reverse');
ylabel('Taxel position x (mm)');
axis equal;
hold off;

if SAVE_FIGURES
    saveas(f2,'Taxel_positions_right_palm_FoR10.fig');
    print -f2 -djpeg 'Taxel_positions_right_palm_FoR10.jpg';
end


f3 = figure(3);
clf(f3);
length = 10; % for ref. frame
title('Taxel positions right palm FoR, with thermal pads and repr. taxels ');
hold on;
% we swap the axes for visualiation to match better with the palm
plot(taxel_positions_FoR_10(:,2),taxel_positions_FoR_10(:,1),'xb');
for i=1:NR_TAXELS
   if ( ((i-1+TAXEL_ID_OFFSET_PALM_TO_HAND) == 107) || ((i-1+TAXEL_ID_OFFSET_PALM_TO_HAND) == 119) || ((i-1+TAXEL_ID_OFFSET_PALM_TO_HAND) == 131) || ((i-1+TAXEL_ID_OFFSET_PALM_TO_HAND) == 139) ) %thermal pads
          text(taxel_positions_FoR_10(i,2),taxel_positions_FoR_10(i,1),int2str(i-1+TAXEL_ID_OFFSET_PALM_TO_HAND),'FontSize',8); 
   elseif ( ((i-1+TAXEL_ID_OFFSET_PALM_TO_HAND) == 101) || ((i-1+TAXEL_ID_OFFSET_PALM_TO_HAND) == 103) || ((i-1+TAXEL_ID_OFFSET_PALM_TO_HAND) == 118) ...
                || ((i-1+TAXEL_ID_OFFSET_PALM_TO_HAND) == 124)  || ((i-1+TAXEL_ID_OFFSET_PALM_TO_HAND) == 137)) %repr. taxels
          text(taxel_positions_FoR_10(i,2),taxel_positions_FoR_10(i,1),int2str(i-1+TAXEL_ID_OFFSET_PALM_TO_HAND),'FontSize',14,'FontWeight','bold'); 
   else
            text(taxel_positions_FoR_10(i,2),taxel_positions_FoR_10(i,1),int2str(i-1+TAXEL_ID_OFFSET_PALM_TO_HAND),'FontSize',14); 
   end
end
h = quiver(0 ,0, 10,0);
set(h, 'Color', 'g', 'LineWidth', 2, 'MaxHeadSize', 4, 'ShowArrowHead', 'on');
text(5,0,'y','FontSize',14);
h2 = quiver(0,0, 0,10);
set(h2, 'Color', 'r', 'LineWidth', 2, 'MaxHeadSize', 4, 'ShowArrowHead', 'on')
text(0,5,'x','FontSize',14);
 

xlabel('Taxel position y (mm)');
set(gca,'XDir','reverse');
ylabel('Taxel position x (mm)');
axis equal;
hold off;

if SAVE_FIGURES
    saveas(f3,'Taxel_positions_right_palm_FoR10_withThermalAndRepTaxels.fig');
    print -f3 -djpeg 'Taxel_positions_right_palm_FoR10_withThermalAndRepTaxels.jpg';
end

%% prepare output

if PRODUCE_OUTPUT_FILE

% convert to meters
for i=1:NR_TAXELS
    taxel_positions_FoR_10_meters = taxel_positions_FoR_10 ./ 1000.0; 
end

%with taxel IDs
taxel_IDs_and_positions_palm_only_FoR_10_meters = [TAXEL_IDS_AND_POSITIONS(:,1) taxel_positions_FoR_10_meters];
taxel_handIDs_and_positions_palm_only_FoR_10_meters = taxel_IDs_and_positions_palm_only_FoR_10_meters;
for i=1:NR_TAXELS
   taxel_handIDs_and_positions_palm_only_FoR_10_meters(i,1) =  taxel_handIDs_and_positions_palm_only_FoR_10_meters(i,1) + TAXEL_ID_OFFSET_PALM_TO_HAND;
end

dlmwrite('right_palm_only_IDs_and_positions_meters.txt', taxel_IDs_and_positions_palm_only_FoR_10_meters);
dlmwrite('right_palm_only_handIDs_and_positions_meters.txt', taxel_handIDs_and_positions_palm_only_FoR_10_meters);

% now prepare the text file with 3 position coordinates and three with the
% normal - we will assign 0 0 -1 - that is point out of the palm,
% with 192 rows - taxel ID is implicit in the (row number - 1)
beginning_zeros=zeros(TAXEL_ID_OFFSET_PALM_TO_HAND,6);
end_zeros_count = 192-(NR_TAXELS+TAXEL_ID_OFFSET_PALM_TO_HAND);
end_zeros=zeros(end_zeros_count,6);
for j=1:NR_TAXELS
    taxel_positions_and_normals_FoR_10_meters(j,:) = [taxel_positions_FoR_10_meters(j,:) 0 0 1 ]; % ! the left palm has the z pointing from the palm (unlike left palm)
end
taxel_positions_and_normals_palm_and_fake_fingers_FoR_10_meters = [beginning_zeros ; taxel_positions_and_normals_FoR_10_meters; end_zeros];
dlmwrite('right_hand.txt',taxel_positions_and_normals_palm_and_fake_fingers_FoR_10_meters,'delimiter', '\t', ...
         'precision', 5);
     
end