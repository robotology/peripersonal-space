% Edited by Francesco Nori and Nikos Tzagarachis
% Genova Jan 2008
% This function computes the iCub right leg forward kinematic as
% described by the CAD files. Parameters are given according to the DH 
% notation. Global forward kinematic for the right leg are given by 
% T_Ro0 * T_0n.
%
% Usage:
%
%       [T_Ro0, T_0n] = WaistLeftArmFwdKin(rtheta)
%
% Input: 
%
%       rtheta = [rtheta0, rtheta1, rtheta2, rtheta3, rtheta4, rtheta5]
%       rtheta0: hip_pitch
%       rtheta1: hip_roll
%       rtheta2: hip_yaw
%       rtheta3: knee
%       rtheta4: ankle_pitch
%       rtheta5: ankle_roll
%
% Output:
%
%       T_Ro0: rigid transformation from root to the 0th reference frame of
%       the DH notation
%       T_0n:  rigid transformation from the 0th reference frame of the DH
%       notation to the last reference frame of the DH notation (right leg)

function [T_Ro0, T_0n, J] = WaistRightLegFwdKin(rtheta, display)

% syms alph a d thet real;
% G_LtoL = [1 0 0 0; 0 cos(alph) -sin(alph) 0; 0 sin(alph) cos(alph) 0; 0 0 0 1];
% G_LtoL = [eye(3, 3), [a 0 0]'; 0 0 0 1] * G_LtoL;
% G_LtoL = [eye(3, 3), [0 0 d]'; 0 0 0 1] * G_LtoL;
% G_LtoL = [cos(thet) -sin(thet) 0  0; sin(thet) cos(thet) 0  0; 0 0 1 0; 0 0 0 1] * G_LtoL;

rtheta0 = rtheta(1);
rtheta1 = rtheta(2);
rtheta2 = rtheta(3);
rtheta3 = rtheta(4);
rtheta4 = rtheta(5);
rtheta5 = rtheta(6);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%            Left Leg                       *
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
RG_01 = evalDHMatrix(     0,        0,     pi/2,      rtheta0 + pi/2);
RG_12 = evalDHMatrix(     0,        0,     pi/2,      rtheta1 + pi/2);
RG_23 = evalDHMatrix(     0,   240.00,    -pi/2,      rtheta2 - pi/2);
RG_34 = evalDHMatrix(-220.0,        0,     pi  ,      rtheta3+pi/2);
RG_45 = evalDHMatrix(     0,        0,     pi/2,      rtheta4);
RG_56 = evalDHMatrix( -41.0,        0,     pi  ,       rtheta5);

RG_sL0 = [ Rx(-pi/2), zeros(3,1); 0 0 0 1]*eye(4);
RG_sL0(1:3,4)=[0  68.1 -119.9]';
RG_sL1 = RG_sL0*RG_01;
RG_sL2 = RG_sL1*RG_12;
RG_sL3 = RG_sL2*RG_23;
RG_sL4 = RG_sL3*RG_34;
RG_sL5 = RG_sL4*RG_45;
RG_sL6 = RG_sL5*RG_56;
XE=[0 0 0 1]';
rx = RG_sL6*XE;

T_Ro0 = RG_sL0;
T_0n = RG_01*RG_12*RG_23*RG_34*RG_45*RG_56;

if nargout == 3
    J = [];
    Pn =     RG_sL6(1:3,4);
    P(:,1) = RG_sL0(1:3,4);
    P(:,2) = RG_sL1(1:3,4);
    P(:,3) = RG_sL2(1:3,4);        
    P(:,4) = RG_sL3(1:3,4);    
    P(:,5) = RG_sL4(1:3,4);
    P(:,6) = RG_sL5(1:3,4);
    
    z(:,1) = RG_sL0(1:3,3);
    z(:,2) = RG_sL1(1:3,3);
    z(:,3) = RG_sL2(1:3,3);        
    z(:,4) = RG_sL3(1:3,3);    
    z(:,5) = RG_sL4(1:3,3);
    z(:,6) = RG_sL5(1:3,3);
    
    for i = 1 : 6
        J = [J [cross(z(:,i), Pn-P(:,i)); z(:,i)]];
    end
end

if display == 1
    %Draw a representation of the kinematic chain
    LinkColor1 = [1 0 0];   %RGB color of the first link
    LinkColor2 = [1 0 0];   %RGB color of the second link
    LinkColor3 = [1 1 0];   %RGB color of the third link
    LinkColor4 = [0 0 1];   %RGB color of the first link


    footx = 1;
    footy = 20;
    footz = 40;

    %right leg
    cy0 = DrawCylinder(-68, 30, RG_sL0*[Rx(0), zeros(3,1); 0 0 0 1], LinkColor2, 100, 0.1);
    hold on
    cy0 = DrawCylinder(-240, 20, RG_sL2*[Ry(-pi), zeros(3,1); 0 0 0 1], LinkColor2, 100, 0.1);
    cy0 = DrawCylinder(-220, 10, RG_sL4*[Ry(-pi/2), zeros(3,1); 0 0 0 1], LinkColor2, 100, 0.1);
    %cy1 = DrawCylinder(-41,   5, RG_sL6*[Ry(-pi/2), zeros(3,1); 0 0 0 1], LinkColor2, 100, 0.1);
    % right foot
    cy1 = DrawRectangle(T_Ro0 * T_0n * [eye(3), [0 0 footz/2]'; 0 0 0 1], 1, 20, 40);

    %
    
    DrawRefFrame(RG_sL0,0)
    DrawRefFrame(RG_sL1,1)
    DrawRefFrame(RG_sL2,2)
    DrawRefFrame(RG_sL3,3)
    DrawRefFrame(RG_sL4,4)
    DrawRefFrame(RG_sL5,5)
    DrawRefFrame(RG_sL6,6)

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    axis('equal')
    xlabel('x')
    ylabel('y')
    zlabel('z')

    L1 = light;
    set(L1, 'Position', [-20 -20 -2])
    L2 = light;
    set(L2, 'Position', [20 20 2])
end