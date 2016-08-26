%% About 
% Script to load taxel position for visualize the PPS of the left forearm of iCub
% Author: NGUYEN Dong Hai Phuong
% Email: phuong.nguyen@iit.it; ph17dn@gmail.com

%%

SAVE_FIGURES = false;
%% Init taxel positions from Andrea's calibration and CAD

load left_forearm_taxel_pos_mesh.mat; % the no_mesh is also possible, but there are no normals, so you can't overlay the triangular modules
taxel_pos = left_forearm_taxel_pos_mesh; 
[M,N] = size(taxel_pos);

load left_forearm_upper_patch_7triangleMidpointsPos_iCubV1_CAD.mat;
triangle_centers_CAD_upperPatch_arbitraryFoR = left_forearm_upper_patch_7triangleMidpointsPos_iCubV1_xmlFoR_m;
triangle_centers_CAD_upperPatch_wristFoR8 = [];

load left_forearm_assembly_18coverHolesPos_iCubV1_CAD.mat;
% note, this CAD data contains also the upper patch - could be used to
% double check
triangle_centers_CAD_lowerPatches_assemblyFoR = left_forearm_lower_patch_18coverHoles_iCubV1_xmlFoR_m  ; % this FoR is also kind of arbitrary, but different than in the previous set;
triangle_centers_CAD_lowerPatches_wristFoR8 = [];

for j=1:size(triangle_centers_CAD_upperPatch_arbitraryFoR,1)
   row_vector =  triangle_centers_CAD_upperPatch_arbitraryFoR(j,:);
   column_vector = row_vector';
   column_vector_translated = column_vector - toWristFoR8_transl_m;
   column_vector_translatedAndRotated = (toWristFoR8_rotMatrix)' * column_vector_translated;
   triangle_centers_CAD_upperPatch_wristFoR8(j,:) = column_vector_translatedAndRotated';     
end

for j=1:size(triangle_centers_CAD_lowerPatches_assemblyFoR,1)
   row_vector =  triangle_centers_CAD_lowerPatches_assemblyFoR(j,:);
   column_vector = row_vector';
   column_vector_translated = column_vector - forearmAssemblytoWristFoR8_translVector_m;
   column_vector_translatedAndRotated = (forearmAssemblytoWristFoR8_rotMatrix)' * column_vector_translated;
   triangle_centers_CAD_lowerPatches_wristFoR8(j,:) = column_vector_translatedAndRotated';     
end

%%

f21 = figure(21);
clf(f21);
title('Positions of foreram taxels with their IDs - lower patch (in 1st wrist FoR - FoR_8)');
hold on;

for i=1:192
    if (nnz(taxel_pos(i,:)) > 1) % it's not an all-zero row
        plot3(taxel_pos(i,1),taxel_pos(i,2),taxel_pos(i,3),'xb');
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

xlabel('Taxel position x (m)');
set(gca,'XDir','reverse');
ylabel('Taxel position y (m)');
zlabel('Taxel position z (m)');
set(gca,'ZDir','reverse');
axis equal;
hold off;
% print(f21,'-dpdf','-bestfit','lower-patch.pdf')


%%
f31 = figure(31);
clf(f31);
title('Positions of foreram taxels with their IDs - upper patch (in 1st wrist FoR - FoR_8)');
hold on;

for i=193:M
    if (nnz(taxel_pos(i,:)) > 1) % it's not an all-zero row
        plot3(taxel_pos(i,1),taxel_pos(i,2),taxel_pos(i,3),'xb');
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

xlabel('Taxel position x (m)');
set(gca,'XDir','reverse');
ylabel('Taxel position y (m)');
zlabel('Taxel position z (m)');
set(gca,'ZDir','reverse');
axis equal;
hold off;

% print(f31,'-dpdf','-bestfit','upper-patch.pdf')