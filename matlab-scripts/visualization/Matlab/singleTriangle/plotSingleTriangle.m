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

rtaxel  =     0.0047/2;              % radius of a single taxel - Ale's measurement

%% Plot canonical triangle

    ang    = 0:0.005:2*pi';
 
    f10 = figure(10);
    clf(f10);
    hold on;
    axis equal;
    for i = 1:length(taxel)
        if (i==7) || (i==11) % thermal pads
            radius = rtaxel/2;
        else
            radius = rtaxel;
        end
        x      = radius*cos(ang) + taxel(i).Pos(1);
        y      = radius*sin(ang) + taxel(i).Pos(2);
        z      = zeros(1,length(ang));
        text(taxel(i).Pos(1),taxel(i).Pos(2),0,int2str(i));
        ht = plot3(x,y,z,'LineWidth',2);
      
    end
    hold off;
   

%% Mirrored triangle

transf_taxels = transformNominalTriangle(1,eye(4), 'true'); 

%% Plot transformed triangle

    ang    = 0:0.005:2*pi';
    f11 = figure(11);
    clf(f11)
    hold on;
    for i = 1:length(transf_taxels)
        if (i==7) || (i==11) % thermal pads
            radius = rtaxel/2;
        else
            radius = rtaxel;
        end
        x      = radius*cos(ang) + transf_taxels(i).Pos(1);
        y      = radius*sin(ang) + transf_taxels(i).Pos(2);
        z      = repmat(transf_taxels(i).Pos(3),1,length(ang));
        text(transf_taxels(i).Pos(1),transf_taxels(i).Pos(2),transf_taxels(i).Pos(3),int2str(transf_taxels(i).ID));
        ht = plot3(x,y,z,'--.r');
      
    end
 
    hold off;
    
% %% Test transformation
% 
% theta = 45;
% Homo_Rot_About_Z = [cos(theta) -sin(theta) 0 0; sin(theta) cos(theta) 0 0; 0 0 1 0; 0 0 0 1];
% Homo_reflectionInX = [-1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1];
% transf_taxels = transformNominalTriangle(288,Homo_reflectionInX*Homo_Rot_About_Z); % *Homo_reflectionInX
% 
% %% Plot transformed triangle
% 
%     ang    = 0:0.005:2*pi';
%     figure(f10);
%     hold on;
%     for i = 1:length(transf_taxels)
%         if (i==7) || (i==11) % thermal pads
%             radius = rtaxel/2;
%         else
%             radius = rtaxel;
%         end
%         x      = radius*cos(ang) + transf_taxels(i).Pos(1);
%         y      = radius*sin(ang) + transf_taxels(i).Pos(2);
%         z      = repmat(transf_taxels(i).Pos(3),1,length(ang));
%         text(transf_taxels(i).Pos(1),transf_taxels(i).Pos(2),transf_taxels(i).Pos(3),int2str(transf_taxels(i).ID));
%         ht = plot3(x,y,z,'--.r');
%       
%     end
%     xlabel('Taxel position x (m)');
%     set(gca,'XDir','reverse');
%     ylabel('Taxel position y (m)');
%     zlabel('Taxel position z (m)');
% 
%     hold off;

    
    %% Test transformation 2 

%theta = 45;
%Homo_Rot_About_Z = [cos(theta) -sin(theta) 0 0; sin(theta) cos(theta) 0 0; 0 0 1 0; 0 0 0 1];
%Homo_reflectionInXAndY = [-1 0 0 0; 0 -1 0 0; 0 0 1 0; 0 0 0 1];
Homo_translation=[1 0 0 0.023; 0 1 0 0; 0 0 1 0; 0 0 0 1];
transf_taxels = transformNominalTriangle(36,Homo_translation); % *Homo_reflectionInX

%% Plot transformed triangle 2

    ang    = 0:0.005:2*pi';
    figure(f10);
    hold on;
    for i = 1:length(transf_taxels)
        if (i==7) || (i==11) % thermal pads
            radius = rtaxel/2;
        else
            radius = rtaxel;
        end
        x      = radius*cos(ang) + transf_taxels(i).Pos(1);
        y      = radius*sin(ang) + transf_taxels(i).Pos(2);
        z      = repmat(transf_taxels(i).Pos(3),1,length(ang));
        text(transf_taxels(i).Pos(1),transf_taxels(i).Pos(2),transf_taxels(i).Pos(3),int2str(transf_taxels(i).ID));
        ht = plot3(x,y,z,'--.r');
      
    end
    xlabel('Taxel position x (m)');
    %set(gca,'XDir','reverse');
    ylabel('Taxel position y (m)');
    zlabel('Taxel position z (m)');

    hold off;
