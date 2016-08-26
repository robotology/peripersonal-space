function DrawLeftHand(G)

load('leftHandMesh.mat');

k = 1/1000;

node_xyz = node_xyz.*.7.*k;
node_xyz(4,:) = 1;

node_xyzG = [Rx(-pi/2) * Ry(pi/2)  [0 0 0]'; 0 0 0 1]  * node_xyz;
alpha = pi;
matT=  [cos(alpha) -sin(alpha) 0 0;
        sin(alpha)  cos(alpha) 0 0;
        0 0 1 0;
        0 0 0 1];

node_xyzG = G * node_xyzG;


h = trisurf(face_node', node_xyzG(1,:), node_xyzG(2,:), node_xyzG(3,:));

set(h, 'EdgeAlpha', 0);
%set(h, 'FaceAlpha', 0.1);

