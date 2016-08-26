function [ taxels_out ] = transformNominalTriangle( start_ID, homo_transform_matrix, vertical_mirror )
%TRANSFORMNOMINALTRIANGLE Summary of this function goes here
%   Detailed explanation goes here

%% Init canonical triangle

%nominal triangle - putting values from /media/DATA/data_work/icub_skin/CDC_PAD_positions.xlsx
% converted to metres
% (iCubSkinGui also has the info - in
% icub-main/src/tools/iCubSkinGui/src/include/Triangle_10pad.h, but in
% strange units)

%PAD POSITION			
%	X	Y	
%P0	6.533	0	
%P1	9.8	-5.66	
%P2	3.267	-5.66	
%P3	0	0	
%P4	-3.267	-5.66	
%P5	-9.8	-5.66	
%P6	-6.51	-3.75	thermalpad
%P7	-6.533	0	
%P8	-3.267	5.66	
%P9	0	11.317	
%P10	0	7.507	thermalpad
%P11	3.267	5.66	
% These seem to be centers of individual pads relative to the central
% pad/taxel - P3, in mm

taxel(1).Pos = [ 0.006533, 0 , 0];
taxel(2).Pos = [ 0.0098, -0.00566 , 0];
taxel(3).Pos = [ 0.003267, -0.00566 , 0];
taxel(4).Pos = [ 0,0, 0];
taxel(5).Pos = [ -0.003267, -0.00566, 0];
taxel(6).Pos = [ -0.0098,-0.00566 , 0];
taxel(7).Pos = [ -0.00651,-0.00375 , 0]; % thermal pad
taxel(8).Pos = [ -0.006533, 0, 0];
taxel(9).Pos = [ -0.003267, 0.00566, 0];
taxel(10).Pos = [0 , 0.011317, 0];
taxel(11).Pos = [0 ,0.007507 , 0]; % thermal pad
taxel(12).Pos = [ 0.003267, 0.00566, 0];


%%
taxels_out=[];

for i=1:length(taxel)
   if vertical_mirror % in x
    taxel(i).Pos(1) =  taxel(i).Pos(1) * (-1);
   end
   taxel(i).PosHomo = [taxel(i).Pos 1];
   taxels_out(i).Pos = (homo_transform_matrix * (taxel(i).PosHomo)')';
   taxels_out(i).Pos(4) = [];
   taxels_out(i).ID = start_ID + i - 1; 
end


end

