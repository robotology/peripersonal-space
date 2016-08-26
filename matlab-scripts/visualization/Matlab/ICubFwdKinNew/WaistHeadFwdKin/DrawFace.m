function h = DrawFace(G)

k = 1/1000;
indices = load('Head_mesh_Indices.ini');
vertices = load('Head_mesh_Vertices.ini');

vertices = vertices.*170.*k;
% vertices(:,4)
vertices(:,4) = 1;
indices = indices+1;

verticesG = [Rz(pi) * Ry(-pi/2)  [60*k -10*k 0]'; 0 0 0 1]  * vertices';
% verticesG = vertices';
alpha = pi;
matT=  [cos(alpha) -sin(alpha) 0 0;
        sin(alpha)  cos(alpha) 0 0;
        0 0 1 0;
        0 0 0 1];
    
verticesG = matT * G * verticesG;
verticesG = verticesG'; 

h = trisurf(indices, verticesG(:,1)', verticesG(:,2)', verticesG(:,3)');

set(h, 'FaceColor', 'm');
set(h, 'EdgeAlpha', 0.05);
set(h, 'FaceAlpha', 0.05);

