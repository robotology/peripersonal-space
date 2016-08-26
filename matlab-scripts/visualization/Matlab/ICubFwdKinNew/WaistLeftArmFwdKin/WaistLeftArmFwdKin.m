% Edited by Francesco Nori
% Genova Jan 2008
% This function computes the iCub left arm forward kinematic as
% described by the CAD files. The kinematics include the 
% iCub waist. Parameters are given according to the DH notation. Global
% forward kinematic is given by T_Ro0 * T_0n.
%
% Usage:
%
%       [T_Ro0, T_0n] = WaistLeftArmFwdKin(wtheta,ltheta)
%
% Input: 
%
%       wtheta = [wtheta0, wtheta1, wtheta2]
%       wtheta0: torso_pitch
%       wtheta1: torso_roll
%       wtheta2: torso_yaw
%
%       ltheta = [ltheta0, ltheta1, ltheta2, ltheta3, ltheta4, ltheta5, ltheta6]
%       ltheta0: shoulder_pitch
%       ltheta1: shoulder_roll
%       ltheta2: shoulder_yaw
%       ltheta3: elbow
%       ltheta4: wrist_prosup
%       ltheta5: wrist_pitch
%       ltheta6: wirst_yaw
%
% Output:
%
%       T_Ro0: rigid transformation from root to the 0th reference frame of
%       the DH notation
%       T_0n:  rigid transformation from the 0th reference frame of the DH
%       notation to the last reference frame of the DH notation
%       

function [T_Ro0, T_0n, J, G_sL8, G_sL10] = WaistLeftArmFwdKin(wtheta,ltheta, display)

k = 1/1000; %Conversion rate from mm to m
% syms alph a d thet real;
% G_LtoL = [1 0 0 0; 0 cos(alph) -sin(alph) 0; 0 sin(alph) cos(alph) 0; 0 0 0 1];
% G_LtoL = [eye(3, 3), [a 0 0]'; 0 0 0 1] * G_LtoL;
% G_LtoL = [eye(3, 3), [0 0 d]'; 0 0 0 1] * G_LtoL;
% G_LtoL = [cos(thet) -sin(thet) 0  0; sin(thet) cos(thet) 0  0; 0 0 1 0; 0 0 0 1] * G_LtoL;

wtheta0 = wtheta(3);
wtheta1 = wtheta(2);
wtheta2 = wtheta(1);

%Reference frames attached to links
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%               Waist                       *
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
G_01=evalDHMatrix(     32*k,       0*k,     pi/2,                wtheta0);
G_12=evalDHMatrix(      0*k,    -5.5*k,     pi/2,                wtheta1-pi/2);
G_23=evalDHMatrix(23.3647*k,  -143.3*k,    -pi/2,                wtheta2 + 15*pi/180 + pi/2 );

%G_sL0 = [ Ry(-pi/2), zeros(3,1); 0 0 0 1]*[Rz(pi), zeros(3,1); 0 0 0 1]*eye(4);  
%G_sL0 = [Ry(pi/2), zeros(3,1); 0 0 0 1]*[Rz(pi)*Rx(-pi/2), zeros(3,1); 0 0 0 1]*eye(4);  
G_sL0 = [Rz(-pi/2) zeros(3,1); zeros(1,3) 1]*[0 0 1 0; 0 -1 0 0; 1 0 0 0; 0 0 0 1];
G_sL1 = G_sL0*G_01;
G_sL2 = G_sL1*G_12;
G_sL3 = G_sL2*G_23;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%               LeftArm                     *
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


theta0 = ltheta(1);
theta1 = ltheta(2);
theta2 = ltheta(3);
theta3 = ltheta(4);
theta4 = ltheta(5);
theta5 = ltheta(6);
theta6 = ltheta(7);

G_34=evalDHMatrix(     0,  107.74*k,    -pi/2, theta0+pi/2);
G_45=evalDHMatrix(     0,         0,     pi/2, theta1-pi/2);
G_56=evalDHMatrix(  15*k,  152.28*k,    -pi/2, theta2+pi/2-15*pi/180); 
G_67=evalDHMatrix( -15*k,         0,     pi/2,  theta3); 
G_78=evalDHMatrix(     0,   137.3*k,     pi/2, theta4-pi/2); 
G_89=evalDHMatrix(     0,         0,     pi/2, theta5+pi/2); 
G_910=evalDHMatrix(62.5*k,    -16*k,        0, theta6); 

%Reference frames attached to links
G_sL4   = G_sL3  * G_34;
G_sL5   = G_sL4  * G_45;
G_sL6   = G_sL5  * G_56;
G_sL7   = G_sL6  * G_67;
G_sL8   = G_sL7  * G_78;
G_sL9   = G_sL8  * G_89;
G_sL10  = G_sL9  * G_910;
XE=[0 0 0 1]';
x = G_sL10*XE;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

T_Ro0 = G_sL0;
T_0n  = G_01*G_12*G_23*G_34*G_45*G_56*G_67*G_78*G_89*G_910;



if nargout == 5
    J = [];
    Pn =      G_sL10(1:3,4);
    P(:,1) = G_sL0(1:3,4);
    P(:,2) = G_sL1(1:3,4);
    P(:,3) = G_sL2(1:3,4);        
    P(:,4) = G_sL3(1:3,4);    
    P(:,5) = G_sL4(1:3,4);
    P(:,6) = G_sL5(1:3,4);
    P(:,7) = G_sL6(1:3,4);
    P(:,8) = G_sL7(1:3,4);    
    P(:,9) = G_sL8(1:3,4);
    P(:,10)= G_sL9(1:3,4);
    
    z(:,1) = G_sL0(1:3,3);
    z(:,2) = G_sL1(1:3,3);
    z(:,3) = G_sL2(1:3,3);        
    z(:,4) = G_sL3(1:3,3);    
    z(:,5) = G_sL4(1:3,3);
    z(:,6) = G_sL5(1:3,3);
    z(:,7) = G_sL6(1:3,3);
    z(:,8) = G_sL7(1:3,3);    
    z(:,9) = G_sL8(1:3,3);
    z(:,10)= G_sL9(1:3,3);
    
    J = [J [cross(z(:,3), Pn-P(:,3)); z(:,3)]];
    J = [J [cross(z(:,2), Pn-P(:,2)); z(:,2)]];
    J = [J [cross(z(:,1), Pn-P(:,1)); z(:,1)]];
    for i = 4 : 10
        J = [J [cross(z(:,i), Pn-P(:,i)); z(:,i)]];
    end
end


if (display==1)
    %Draw a representation of the kinematic chain
    ljnt = 20*k;  %joint pic length
    rjnt = 5*k; %joint pic radius
    LinkColor1 = [1 0 0];   %RGB color of the first link
    LinkColor2 = [1 1 0];   %RGB color of the second link
    LinkColor3 = [0 0 0];   %RGB color of the third link
    JntColor   = [.7 .7 .7];%RGB color of the joints

    jnt2 = DrawCylinder(ljnt, rjnt, G_sL0*[1 0 0 0; 0 1 0 0; 0 0 1 -ljnt/2; 0 0 0 1], JntColor, 100, 0.5);
    hold on
    jnt3 = DrawCylinder(ljnt, rjnt, G_sL1*[1 0 0 0; 0 1 0 0; 0 0 1 -ljnt/2; 0 0 0 1], JntColor, 100, 0.5);
    jnt4 = DrawCylinder(ljnt, rjnt, G_sL2*[1 0 0 0; 0 1 0 0; 0 0 1 -ljnt/2; 0 0 0 1], JntColor, 100, 0.5);
    %jnt5 = DrawCylinder(ljnt, rjnt, WG_sL3*[1 0 0 0; 0 1 0 0; 0 0 1 -ljnt/2; 0 0 0 1], JntColor, 100, 0.5);

    DrawRefFrame(G_sL0,0)
    DrawRefFrame(G_sL1,1)
    DrawRefFrame(G_sL2,2)

    % jnt1 = DrawCylinder(ljnt, rjnt, G_sL0 * [1 0 0 0; 0 1 0 0; 0 0 1 -ljnt/2; 0 0 0 1], JntColor, 100, 0.8);
    jnt2 = DrawCylinder(ljnt, rjnt, G_sL3*[1 0 0 0; 0 1 0 0; 0 0 1 -ljnt/2; 0 0 0 1], JntColor, 100, 0.5);
    hold on
    jnt3 = DrawCylinder(ljnt, rjnt, G_sL4*[1 0 0 0; 0 1 0 0; 0 0 1 -ljnt/2; 0 0 0 1], JntColor, 100, 0.5);
    jnt4 = DrawCylinder(ljnt, rjnt, G_sL5*[1 0 0 0; 0 1 0 0; 0 0 1 -ljnt/2; 0 0 0 1], JntColor, 100, 0.5);
    jnt5 = DrawCylinder(ljnt, rjnt, G_sL6*[1 0 0 0; 0 1 0 0; 0 0 1 -ljnt/2; 0 0 0 1], JntColor, 100, 0.5);
    jnt6 = DrawCylinder(ljnt, rjnt, G_sL7*[1 0 0 0; 0 1 0 0; 0 0 1 -ljnt/2; 0 0 0 1], JntColor, 100, 0.5);
    jnt7 = DrawCylinder(ljnt, rjnt, G_sL8*[1 0 0 0; 0 1 0 0; 0 0 1 -ljnt/2; 0 0 0 1], JntColor, 100, 0.5);
    jnt8 = DrawCylinder(ljnt, rjnt, G_sL9*[1 0 0 0; 0 1 0 0; 0 0 1 -ljnt/2; 0 0 0 1], JntColor, 100, 0.5);
    % %
    cy0 = DrawCylinder(107.74*k, 20*k, G_sL3, LinkColor3, 100, 0.7);
    cy0 = DrawCylinder(152.28*k, 10*k, G_sL5, LinkColor3, 100, 0.7);
    cy1 = DrawCylinder(137.3*k, 5*k,   G_sL7, LinkColor3, 100, 0.7);
    % Palm
    %cy2 = DrawRectangle(G_sL10*[eye(3), [-42.5/2 0 0]'; 0 0 0 1],  42.5, 30, 10);

    DrawRefFrame(G_sL3,3)
    DrawRefFrame(G_sL4,4)
    DrawRefFrame(G_sL5,5)
    DrawRefFrame(G_sL6,6)
    DrawRefFrame(G_sL7,7)
    DrawRefFrame(G_sL8,8)
    DrawRefFrame(G_sL9,9)
    DrawRefFrame(G_sL10,10)
    %DrawRefFrame(G_sL10*[Ry(pi) zeros(3,1); zeros(1,3) 1],11)
    
    %Draw the hand
%     DrawLeftHand(G_sL10);

%     axis('equal')
%     xlabel('x')
%     ylabel('y')
%     zlabel('z')

%     L1 = light;
%     set(L1, 'Position', [-20 -20 -2])
%     L2 = light;
%     set(L2, 'Position', [20 20 2])
end



