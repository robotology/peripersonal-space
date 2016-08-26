% Edited by Francesco Nori and Nikos Tzagarachis
% Genova Jan 2008
% This function computes the iCub left leg forward kinematic as
% described by the CAD files. Parameters are given according to the DH 
% notation. Global forward kinematic for the left leg are given by 
% T_Ro0 * T_0n.
%
% Usage:
%
%       [T_Ro0, T_0n] = WaistLeftArmFwdKin(ltheta)
%
% Input: 
%
%       ltheta = [ltheta0, ltheta1, ltheta2, ltheta3, ltheta4, ltheta5]
%       ltheta0: hip_pitch
%       ltheta1: hip_roll
%       ltheta2: hip_yaw
%       ltheta3: knee
%       ltheta4: ankle_pitch
%       ltheta5: ankle_roll
%
% Output:
%
%       T_Ro0: rigid transformation from root to the 0th reference frame of
%       the DH notation
%       T_0n:  rigid transformation from the 0th reference frame of the DH
%       notation to the last reference frame of the DH notation (left leg)

function [T_Ro0, T_0n, J] = WaistLeftLegFwdKin(ltheta, display)

% syms alph a d thet real;
% G_LtoL = [1 0 0 0; 0 cos(alph) -sin(alph) 0; 0 sin(alph) cos(alph) 0; 0 0 0 1];
% G_LtoL = [eye(3, 3), [a 0 0]'; 0 0 0 1] * G_LtoL;
% G_LtoL = [eye(3, 3), [0 0 d]'; 0 0 0 1] * G_LtoL;
% G_LtoL = [cos(thet) -sin(thet) 0  0; sin(thet) cos(thet) 0  0; 0 0 1 0; 0 0 0 1] * G_LtoL;

ltheta0 = ltheta(1);
ltheta1 = ltheta(2);
ltheta2 = ltheta(3);
ltheta3 = ltheta(4);
ltheta4 = ltheta(5);
ltheta5 = ltheta(6);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%            Left Leg                       *
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% LT1=subs(G_LtoL, {a d alph thet}, {      0       0    -pi/2      ltheta0+pi/2});
% LT2=subs(G_LtoL, {a d alph thet}, {      0       0    -pi/2      ltheta1+pi/2});
% LT3=subs(G_LtoL, {a d alph thet}, {      0  -240.00     pi/2      ltheta2+pi/2});
% LT4=subs(G_LtoL, {a d alph thet}, {  -220.0       0     pi         ltheta3+pi/2});
% LT5=subs(G_LtoL, {a d alph thet}, {      0       0     pi/2      ltheta4}); 
% LT6=subs(G_LtoL, {a d alph thet}, {  -41.0       0      0         ltheta5}); 

LG_01 = evalDHMatrix(     0,       0,    -pi/2,      ltheta0+pi/2);
LG_12 = evalDHMatrix(     0,       0,    -pi/2,      ltheta1+pi/2);
LG_23 = evalDHMatrix(     0, -240.00,     pi/2,      ltheta2-pi/2);
LG_34 = evalDHMatrix(-220.0,       0,     pi  ,       ltheta3+pi/2);
LG_45 = evalDHMatrix(     0,       0,     -pi/2,      ltheta4);
LG_56 = evalDHMatrix( -41.0,       0,        0,         ltheta5);

LG_sL0 = [ Rx(-pi/2), zeros(3,1); 0 0 0 1]*eye(4);
LG_sL0(1:3,4)=[0  -68.1 -119.9]';
LG_sL1 = LG_sL0*LG_01;
LG_sL2 = LG_sL1*LG_12;
LG_sL3 = LG_sL2*LG_23;
LG_sL4 = LG_sL3*LG_34;
LG_sL5 = LG_sL4*LG_45;
LG_sL6 = LG_sL5*LG_56;
XE=[0 0 0 1]';
lx = LG_sL6*XE;

T_Ro0 = LG_sL0;
T_0n = LG_01*LG_12*LG_23*LG_34*LG_45*LG_56;

if nargout == 3
    J = [];
    Pn =     LG_sL6(1:3,4);
    P(:,1) = LG_sL0(1:3,4);
    P(:,2) = LG_sL1(1:3,4);
    P(:,3) = LG_sL2(1:3,4);        
    P(:,4) = LG_sL3(1:3,4);    
    P(:,5) = LG_sL4(1:3,4);
    P(:,6) = LG_sL5(1:3,4);
    
    z(:,1) = LG_sL0(1:3,3);
    z(:,2) = LG_sL1(1:3,3);
    z(:,3) = LG_sL2(1:3,3);        
    z(:,4) = LG_sL3(1:3,3);    
    z(:,5) = LG_sL4(1:3,3);
    z(:,6) = LG_sL5(1:3,3);
    
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

    % %left leg
    cy0 = DrawCylinder(68,  30, LG_sL0, LinkColor2, 100, 0.1);
    hold on
    cy0 = DrawCylinder(-240, 20, LG_sL2, LinkColor2, 100, 0.1);
    cy0 = DrawCylinder(-220, 10, LG_sL4*[Ry(-pi/2), zeros(3,1); 0 0 0 1], LinkColor2, 100, 0.1);
    cy1 = DrawCylinder(-41,  5, LG_sL6*[Ry(-pi/2), zeros(3,1); 0 0 0 1], LinkColor2, 100, 0.1);
    % %left foot
    cy1 = DrawRectangle(T_Ro0 * T_0n * [eye(3), [0 0 footz/2]'; 0 0 0 1], 1, 20, 40);
    %
    %
    DrawRefFrame(LG_sL0,0)
    hold on
    DrawRefFrame(LG_sL1,1)
    DrawRefFrame(LG_sL2,2)
    DrawRefFrame(LG_sL3,3)
    DrawRefFrame(LG_sL4,4)
    DrawRefFrame(LG_sL5,5)
    DrawRefFrame(LG_sL6,6)

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