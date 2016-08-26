% Edited by Francesco Nori
% Genova Jan 2008
% This function computes the iCub head forward kinematic as
% described by the CAD files. The kinematics include the 
% iCub waist. Parameters are given according to the DH notation. Global
% forward kinematic for the right eye are given by T_Ro0 * T_0n. Global
% forward kinematic for the left eye are given by T_Ro0 * Tp_0n.
%
% Usage:
%
%       [T_Ro0, T_0n, Tp_0n] = WaistLeftArmFwdKin(wtheta,htheta)
%
% Input: 
%
%       wtheta = [wtheta0, wtheta1, wtheta2]
%       wtheta0: torso_pitch
%       wtheta1: torso_roll
%       wtheta2: torso_yaw
%
%       htheta = [htheta0, htheta1, htheta2, htheta3, htheta4, htheta5]
%       htheta0: neck_pitch
%       htheta1: neck_roll
%       htheta2: neck_yaw
%       htheta3: eyes_tilt
%       htheta4: eyes_version
%       htheta5: eyes_vergence
%
% Output:
%
%       T_Ro0: rigid transformation from root to the 0th reference frame of
%       the DH notation
%       T_0n:  rigid transformation from the 0th reference frame of the DH
%       notation to the last reference frame of the DH notation (right eye)
%       Tp_0n: rigid transformation from the 0th reference frame of the DH
%       notation to the last reference frame of the DH notation (left eye)


function [T_Ro0, T_0n, Tp_0n, J, Jp] = WaistHeadFwdKin(wtheta,htheta, display)

k = 1/1000; %Conversion rate from mm to m
wtheta0 = wtheta(3);
wtheta1 = wtheta(2);
wtheta2 = wtheta(1);

%Reference frames attached to links
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%               Waist                       *
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% G_01=subs(G_LtoL, {a d alph thet}, {     32       0     pi/2                wtheta0});
% G_12=subs(G_LtoL, {a d alph thet}, {      0       0     pi/2                wtheta1-pi/2});
% G_23=subs(G_LtoL, {a d alph thet}, {    2.31   -193.3    -pi/2              wtheta2-pi/2 });

G_01=evalDHMatrix(     32*k,       0*k,     pi/2,                wtheta0);
G_12=evalDHMatrix(      0*k,    -5.5*k,     pi/2,                wtheta1 - pi/2);
G_23=evalDHMatrix(   2.31*k,  -193.3*k,    -pi/2,                wtheta2 - pi/2 );


G_sL0 = [Rz(-pi/2) zeros(3,1); zeros(1,3) 1]*[Ry(pi/2), zeros(3,1); 0 0 0 1]*[Rz(pi), zeros(3,1); 0 0 0 1]*eye(4);  
G_sL1 = G_sL0*G_01;
G_sL2 = G_sL1*G_12;
G_sL3 = G_sL2*G_23;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%               Head                        *
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


theta0 = htheta(1);
theta1 = htheta(2);
theta2 = htheta(3);
theta3 = htheta(4);
theta4 =  (- htheta(6) + htheta(5))/2;             %conversion from verg and vers to right eye pan
theta5 =  (htheta(6) + htheta(5))/2;             %conversion from verg and vers to left eye pan

% G_34=subs(G_LtoL, {a d alph thet}, {  33      0     pi/2      theta0 + pi/2});
% G_45=subs(G_LtoL, {a d alph thet}, {   0      0     pi/2      theta1 + pi/2});
% G_56=subs(G_LtoL, {a d alph thet}, { -54   82.5    -pi/2      theta2 - pi/2}); 
% G_67=subs(G_LtoL, {a d alph thet}, {   0     34    -pi/2      theta3}); 
% G_78=subs(G_LtoL, {a d alph thet}, {   0      0     pi/2      theta4 - pi/2});

G_34=evalDHMatrix(    33*k,      0*k,     pi/2,      theta0 + pi/2);
G_45=evalDHMatrix(     0*k,      1*k,    -pi/2,      theta1 - pi/2 ); 
G_56=evalDHMatrix(   -54*k,   82.5*k,    -pi/2,      theta2 + pi/2) ; 
G_67=evalDHMatrix(     0*k,     34*k,    -pi/2,      theta3); 
G_78=evalDHMatrix(     0*k,      0*k,     pi/2,      theta4 - pi/2); 

Gp_67=evalDHMatrix(    0*k,    -34*k,    -pi/2,      theta3); 
Gp_78=evalDHMatrix(    0*k,      0*k,     pi/2,      theta5-pi/2); 


%Reference frames attached to links
G_sL4   = G_sL3  * G_34;
G_sL5   = G_sL4  * G_45;
G_sL6   = G_sL5  * G_56;
G_sL7   = G_sL6  * G_67;
G_sL8   = G_sL7  * G_78;

Gp_sL7  = G_sL6   * Gp_67;
Gp_sL8  = Gp_sL7  * Gp_78;

T_Ro0 = G_sL0;
T_0n  = G_01*G_12*G_23*G_34*G_45*G_56* G_67* G_78;
Tp_0n = G_01*G_12*G_23*G_34*G_45*G_56*Gp_67*Gp_78;

if nargout >= 4
    Pn =      G_sL8(1:3,4);
    P(:,1) = G_sL0(1:3,4);
    P(:,2) = G_sL1(1:3,4);
    P(:,3) = G_sL2(1:3,4);        
    P(:,4) = G_sL3(1:3,4);    
    P(:,5) = G_sL4(1:3,4);
    P(:,6) = G_sL5(1:3,4);
    P(:,7) = G_sL6(1:3,4);
    P(:,8) = G_sL7(1:3,4);    
    
    z(:,1) = G_sL0(1:3,3);
    z(:,2) = G_sL1(1:3,3);
    z(:,3) = G_sL2(1:3,3);        
    z(:,4) = G_sL3(1:3,3);    
    z(:,5) = G_sL4(1:3,3);
    z(:,6) = G_sL5(1:3,3);
    z(:,7) = G_sL6(1:3,3);
    z(:,8) = G_sL7(1:3,3);    
    
    J = [];
    J = [J [cross(z(:,3), Pn-P(:,3)); z(:,3)]];
    J = [J [cross(z(:,2), Pn-P(:,2)); z(:,2)]];
    J = [J [cross(z(:,1), Pn-P(:,1)); z(:,1)]];
    for i = 4 : 8
        J = [J [cross(z(:,i), Pn-P(:,i)); z(:,i)]];
    end
    
    Pn =      Gp_sL8(1:3,4);
    P(:,8) =  Gp_sL7(1:3,4);    
    z(:,8) =  Gp_sL7(1:3,3);    
    
    Jp = [];
    Jp = [Jp [cross(z(:,3), Pn-P(:,3)); z(:,3)]];
    Jp = [Jp [cross(z(:,2), Pn-P(:,2)); z(:,2)]];
    Jp = [Jp [cross(z(:,1), Pn-P(:,1)); z(:,1)]];
    for i = 4 : 8
        Jp = [Jp [cross(z(:,i), Pn-P(:,i)); z(:,i)]];
    end
end

if (display == 1)
    
    %Draw a representation of the kinematic chain
    ljnt = 20*k;  %joint pic length
    rjnt =  5*k; %joint pic radius
    LinkColor1 = [1 0 0];   %RGB color of the first link
    LinkColor2 = [0 1 1];   %RGB color of the second link
    LinkColor3 = [0 0 0];   %RGB color of the third link
    JntColor   = [.7 .7 .7];%RGB color of the joints
    CamColor   = [ 0 0 0];%RGB color of the joints

    
    jnt2 = DrawCylinder(ljnt, rjnt, G_sL0*[1 0 0 0; 0 1 0 0; 0 0 1 -ljnt/2; 0 0 0 1], JntColor, 100, 0.5);
    hold on
    jnt3 = DrawCylinder(ljnt, rjnt, G_sL1*[1 0 0 0; 0 1 0 0; 0 0 1 -ljnt/2; 0 0 0 1], JntColor, 100, 0.5);
    jnt4 = DrawCylinder(ljnt, rjnt, G_sL2*[1 0 0 0; 0 1 0 0; 0 0 1 -ljnt/2; 0 0 0 1], JntColor, 100, 0.5);

    DrawRefFrame(G_sL0,0)
    DrawRefFrame(G_sL1,1)
    DrawRefFrame(G_sL2,2)
    
    
    %Draw joints
    jnt3 = DrawCylinder(ljnt, rjnt, G_sL3*[1 0 0 0; 0 1 0 0; 0 0 1 -ljnt/2; 0 0 0 1], JntColor, 100, 0.5);
    hold on
    jnt4 = DrawCylinder(ljnt, rjnt, G_sL4*[1 0 0 0; 0 1 0 0; 0 0 1 -ljnt/2; 0 0 0 1], JntColor, 100, 0.5);
    jnt5 = DrawCylinder(ljnt, rjnt, G_sL5*[1 0 0 0; 0 1 0 0; 0 0 1 -ljnt/2; 0 0 0 1], JntColor, 100, 0.5);
    jnt6 = DrawCylinder(ljnt, rjnt, G_sL6*[1 0 0 0; 0 1 0 0; 0 0 1 -ljnt/2; 0 0 0 1], JntColor, 100, 0.5);
    jnt7 = DrawCylinder(ljnt, rjnt, G_sL7*[1 0 0 0; 0 1 0 0; 0 0 1 -ljnt/2; 0 0 0 1], JntColor, 100, 0.5);
    jnt8 = DrawCylinder(ljnt, rjnt, G_sL8*[1 0 0 0; 0 1 0 0; 0 0 1 -ljnt/2; 0 0 0 1], JntColor, 100, 0.5);

    %Draw eye-balls
    eye1 = DrawEye(15*k,  G_sL7, LinkColor2, 100, 0.3);
    eye1 = DrawEye(15*k, Gp_sL7, LinkColor2, 100, 0.3);

    %Draw cameras
    cy2 = DrawCylinder(5*k, 5*k,  G_sL8*[Ry(0), [ 0 0 15*k]'; 0 0 0 1], CamColor, 10, 1);
    cy2 = DrawCylinder(5*k, 5*k, Gp_sL8*[Ry(0), [ 0 0 15*k]'; 0 0 0 1], CamColor, 10, 1);

    DrawRefFrame(G_sL0,0)
    DrawRefFrame(G_sL3,3)
    DrawRefFrame(G_sL4,4)
    DrawRefFrame(G_sL5,5)
    DrawRefFrame(G_sL6,6)
    
    % Draw RoF of eyes and camera
%     DrawRefFrame(G_sL7,7)
%     DrawRefFrame(G_sL8,8)
%     DrawRefFrame(Gp_sL7,7)
%     DrawRefFrame(Gp_sL8,8)

    %Draw the mesh of the face
    DrawFace(G_sL6);

%     DrawRefFrame(G_sL10,10)

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%     axis('equal')
%     xlabel('x')
%     ylabel('y')
%     zlabel('z')
% 
%     L1 = light;
%     set(L1, 'Position', [-20 -20 -2])
%     L2 = light;
%     set(L2, 'Position', [20 20 2])
end
