%% About 
% Script to load taxel position for visualize the PPS of the left palm of iCub
% Author: NGUYEN Dong Hai Phuong
% Email: phuong.nguyen@iit.it; ph17dn@gmail.com

%%

SAVE_FIGURES = false;
PRODUCE_OUTPUT_FILE = false;

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

% % plot original taxel positions
% f1 = figure;
% clf(f1);
% 
% hold on;
% plot(TAXEL_IDS_AND_POSITIONS(:,2),TAXEL_IDS_AND_POSITIONS(:,3),'.b');
% for i=1:NR_TAXELS
%    text(TAXEL_IDS_AND_POSITIONS(i,2),TAXEL_IDS_AND_POSITIONS(i,3),int2str(TAXEL_IDS_AND_POSITIONS(i,1))); 
% end
% xlim([-35 35]);
% ylim([0 70]);
% xlabel('Taxel position "x" (mm)');
% ylabel('Taxel position "y" (mm)');
% axis equal;
% hold off;
% 
% f2 = figure;
% clf(f2);
% Length = 10; % for ref. frame
% title('Taxel positions left palm FoR (nr. 10 in arm kinematics)');
% hold on;
% % we swap the axes for visualiation to match better with the palm
% plot(taxel_positions_FoR_10(:,2),taxel_positions_FoR_10(:,1),'xb');
% for i=1:NR_TAXELS
%    text(taxel_positions_FoR_10(i,2),taxel_positions_FoR_10(i,1),int2str(i-1+TAXEL_ID_OFFSET_PALM_TO_HAND)); 
% end
% h = quiver(0 ,0, 10,0);
% set(h, 'Color', 'g', 'LineWidth', 2, 'MaxHeadSize', 4, 'ShowArrowHead', 'on');
% text(5,0,'y');
% h2 = quiver(0,0, 0,10);
% set(h2, 'Color', 'r', 'LineWidth', 2, 'MaxHeadSize', 4, 'ShowArrowHead', 'on')
% text(0,5,'x');
% 
% ylim([-45 15]);
% xlim([-30 30]);
% xlabel('Taxel position y (mm)');
% ylabel('Taxel position x (mm)');
% axis equal;
% hold off;
% 
% if SAVE_FIGURES
%     saveas(f2,'Taxel_positions_left_palm_FoR10.fig');
%     print -f2 -djpeg 'Taxel_positions_left_palm_FoR10.jpg';
% end    
% 
% f3 = figure;
% clf(f3);
% Length = 10; % for ref. frame
% title('Taxel positions left palm FoR, with thermal pads and repr. taxels ');
% hold on;
% % we swap the axes for visualiation to match better with the palm
% plot(taxel_positions_FoR_10(:,2),taxel_positions_FoR_10(:,1),'xb');
% for i=1:NR_TAXELS
%    if ( ((i-1+TAXEL_ID_OFFSET_PALM_TO_HAND) == 107) || ((i-1+TAXEL_ID_OFFSET_PALM_TO_HAND) == 119) || ((i-1+TAXEL_ID_OFFSET_PALM_TO_HAND) == 131) || ((i-1+TAXEL_ID_OFFSET_PALM_TO_HAND) == 139) ) %thermal pads
%           text(taxel_positions_FoR_10(i,2),taxel_positions_FoR_10(i,1),int2str(i-1+TAXEL_ID_OFFSET_PALM_TO_HAND),'FontSize',8); 
%    elseif ( ((i-1+TAXEL_ID_OFFSET_PALM_TO_HAND) == 99) || ((i-1+TAXEL_ID_OFFSET_PALM_TO_HAND) == 101) || ((i-1+TAXEL_ID_OFFSET_PALM_TO_HAND) == 109) ...
%                 || ((i-1+TAXEL_ID_OFFSET_PALM_TO_HAND) == 122)  || ((i-1+TAXEL_ID_OFFSET_PALM_TO_HAND) == 134)) %repr. taxels
%           text(taxel_positions_FoR_10(i,2),taxel_positions_FoR_10(i,1),int2str(i-1+TAXEL_ID_OFFSET_PALM_TO_HAND),'FontSize',14,'FontWeight','bold'); 
%    else
%             text(taxel_positions_FoR_10(i,2),taxel_positions_FoR_10(i,1),int2str(i-1+TAXEL_ID_OFFSET_PALM_TO_HAND),'FontSize',14); 
%    end
% end
% h = quiver(0 ,0, 10,0);
% set(h, 'Color', 'g', 'LineWidth', 2, 'MaxHeadSize', 4, 'ShowArrowHead', 'on');
% text(5,0,'y','FontSize',14);
% h2 = quiver(0,0, 0,10);
% set(h2, 'Color', 'r', 'LineWidth', 2, 'MaxHeadSize', 4, 'ShowArrowHead', 'on')
% text(0,5,'x','FontSize',14);
%  
% ylim([-45 15]);
% xlim([-30 30]);
% xlabel('Taxel position y (mm)');
% ylabel('Taxel position x (mm)');
% axis equal;
% hold off;
% 
% if SAVE_FIGURES
%     saveas(f3,'Taxel_positions_left_palm_FoR10_withThermalAndRepTaxels.fig');
%     print -f3 -djpeg 'Taxel_positions_left_palm_FoR10_withThermalAndRepTaxels.jpg';
% end
% %f3=figure(3);
% %annotation('arrow',[0 1],[0 0]);
% %quiver(0,0,1,0);
% %plot(0.9,0.1)
% 
% %% prepare output
% 
% if PRODUCE_OUTPUT_FILE
% 
%     % convert to meters
%     for i=1:NR_TAXELS
%         taxel_positions_FoR_10_meters = taxel_positions_FoR_10 ./ 1000.0; 
%     end
% 
%     %with taxel IDs
%     taxel_IDs_and_positions_palm_only_FoR_10_meters = [TAXEL_IDS_AND_POSITIONS(:,1) taxel_positions_FoR_10_meters];
%     taxel_handIDs_and_positions_palm_only_FoR_10_meters = taxel_IDs_and_positions_palm_only_FoR_10_meters;
%     for i=1:NR_TAXELS
%        taxel_handIDs_and_positions_palm_only_FoR_10_meters(i,1) =  taxel_handIDs_and_positions_palm_only_FoR_10_meters(i,1) + TAXEL_ID_OFFSET_PALM_TO_HAND;
%     end
% 
%     dlmwrite('left_palm_only_IDs_and_positions_meters.txt', taxel_IDs_and_positions_palm_only_FoR_10_meters);
%     dlmwrite('left_palm_only_handIDs_and_positions_meters.txt', taxel_handIDs_and_positions_palm_only_FoR_10_meters);
% 
%     % now prepare the text file with 3 position coordinates and three with the
%     % normal - we will assign 0 0 -1 - that is point out of the palm,
%     % with 192 rows - taxel ID is implicit in the (row number - 1)
%     beginning_zeros=zeros(TAXEL_ID_OFFSET_PALM_TO_HAND,6);
%     end_zeros_count = 192-(NR_TAXELS+TAXEL_ID_OFFSET_PALM_TO_HAND);
%     end_zeros=zeros(end_zeros_count,6);
%     for j=1:NR_TAXELS
%         taxel_positions_and_normals_FoR_10_meters(j,:) = [taxel_positions_FoR_10_meters(j,:) 0 0 -1 ];
%     end
%     taxel_positions_and_normals_palm_and_fake_fingers_FoR_10_meters = [beginning_zeros ; taxel_positions_and_normals_FoR_10_meters; end_zeros];
%     dlmwrite('left_hand.txt',taxel_positions_and_normals_palm_and_fake_fingers_FoR_10_meters,'delimiter', '\t', ...
%              'precision', 5);
% end