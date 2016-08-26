% Edited by Francesco Nori
% Genova Jan 2008
% This function computes the iCub inertia sensot forward kinematic as
% described by the CAD files. The kinematics include the 
% iCub waist. Parameters are given according to the DH notation. Global
% forward kinematic for the inertia sensor are given by T_Ro0 * T_0n.
%
% Usage:
%
%       [T_Ro0, T_0n] = WaistLeftArmFwdKin(wtheta,htheta)
%
% Input: 
%
%       wtheta = [wtheta0, wtheta1, wtheta2]
%       wtheta0: torso_pitch
%       wtheta1: torso_roll
%       wtheta2: torso_yaw
%
%       htheta = [htheta0, htheta1, htheta2]
%       htheta0: neck_pitch
%       htheta1: neck_roll
%       htheta2: neck_yaw
%
% Output:
%
%       T_Ro0: rigid transformation from root to the 0th reference frame of
%       the DH notation
%       T_0n:  rigid transformation from the 0th reference frame of the DH
%       notation to the last reference frame of the DH notation (inertia sensor)


function [T_Ro0, T_0I, J] = WaistInertiaFwdKin(wtheta,htheta, display)

k = 1/1000; %Conversion rate from mm to m
wtheta0 = wtheta(3);
wtheta1 = wtheta(2);
wtheta2 = wtheta(1);

%Reference frames attached to links
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%               Waist                       *
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
G_01=evalDHMatrix(     32*k,         0,     pi/2,                wtheta0);
G_12=evalDHMatrix(        0,    -5.5*k,     pi/2,                wtheta1 - pi/2);
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

% NEW DH
G_34=evalDHMatrix(     33*k,         0,      pi/2,      theta0 + pi/2);
G_45=evalDHMatrix(        0,       1*k,     -pi/2,      theta1 - pi/2 );
G_56=evalDHMatrix(   22.5*k,   100.5*k,     -pi/2,      theta2 + pi/2); 
G_6I=evalDHMatrix(        0,     6.6*k,      pi/2,      0); 


%Reference frames attached to links

G_sL4   = G_sL3  * G_34;
G_sL5   = G_sL4  * G_45;
G_sL6   = G_sL5  * G_56;
G_sLI   = G_sL6  * G_6I;

% Transformation from the inertia sensor to the nth reference frame
Inertiax = (53-19)*k;
Inertiay = (38-24.8)*k;
Inertiaz = (20.9-9.5)*k;
G_sI = G_sL6 * G_6I;

if nargout >= 3
    Pn =      G_sI(1:3,4);
    P(:,1) = G_sL0(1:3,4);
    P(:,2) = G_sL1(1:3,4);
    P(:,3) = G_sL2(1:3,4);        
    P(:,4) = G_sL3(1:3,4);    
    P(:,5) = G_sL4(1:3,4);
    P(:,6) = G_sL5(1:3,4);
    
    z(:,1) = G_sL0(1:3,3);
    z(:,2) = G_sL1(1:3,3);
    z(:,3) = G_sL2(1:3,3);        
    z(:,4) = G_sL3(1:3,3);    
    z(:,5) = G_sL4(1:3,3);
    z(:,6) = G_sL5(1:3,3);
    
    J = [];
    J = [J [cross(z(:,3), Pn-P(:,3)); z(:,3)]];
    J = [J [cross(z(:,2), Pn-P(:,2)); z(:,2)]];
    J = [J [cross(z(:,1), Pn-P(:,1)); z(:,1)]];
    for i = 4 : 6
        J = [J [cross(z(:,i), Pn-P(:,i)); z(:,i)]];
    end
end

%eyes fwd kinematics (just for visualization):
G_56_RE=evalDHMatrix(   -54*k,   82.5*k,    -pi/2,       + pi/2) ; %NEW
G_67_RE=evalDHMatrix(     0*k,     34*k,    -pi/2,      0); 
G_78_RE=evalDHMatrix(     0*k,      0*k,     pi/2,       - pi/2); 

G_67_LE=evalDHMatrix(     0*k,    -34*k,    -pi/2,      0); 
G_78_LE=evalDHMatrix(     0*k,      0*k,     pi/2,      -pi/2); 


G_sL6_E    = G_sL5  * G_56_RE;
G_sL7_RE   = G_sL6_E  * G_67_RE;
G_sL8_RE   = G_sL7_RE  * G_78_RE;

G_sL7_LE   = G_sL6_E  * G_67_LE;
G_sL8_LE   = G_sL7_LE  * G_78_LE;

if display==1
    
    %Draw a representation of the kinematic chain
    ljnt = 20*k;  %joint pic length
    rjnt = 5*k; %joint pic radius
    LinkColor1 = [1 0 0];   %RGB color of the first link
    LinkColor2 = [0 1 1];   %RGB color of the second link
    LinkColor3 = [0 0 0];   %RGB color of the third link
    JntColor   = [.7 .7 .7];%RGB color of the joints
    CamColor   = [ 0.2 0.2 0.2];%RGB color of the joints

    
    jnt2 = DrawCylinder(ljnt, rjnt, G_sL0*[1 0 0 0; 0 1 0 0; 0 0 1 -ljnt/2; 0 0 0 1], JntColor, 100, 0.5);
    hold on
    jnt3 = DrawCylinder(ljnt, rjnt, G_sL1*[1 0 0 0; 0 1 0 0; 0 0 1 -ljnt/2; 0 0 0 1], JntColor, 100, 0.5);
    jnt4 = DrawCylinder(ljnt, rjnt, G_sL2*[1 0 0 0; 0 1 0 0; 0 0 1 -ljnt/2; 0 0 0 1], JntColor, 100, 0.5);

    DrawRefFrame(G_sL0,0)
    DrawRefFrame(G_sL1,1)
    DrawRefFrame(G_sL2,2)

    %Draw sensor
    HandleCyl = DrawRectangle(G_sLI * [eye(3), [-Inertiax/2 -Inertiay/2 -Inertiaz/2]'; 0 0 0 1], Inertiax, Inertiay, Inertiaz);

    DrawRefFrame(G_sL3,3)
    DrawRefFrame(G_sL4,4)
    DrawRefFrame(G_sL5,5)
    DrawRefFrame(G_sLI,6)
    
    %Draw eye-balls
    eye1 = DrawEye(15*k,  G_sL7_RE, LinkColor2, 100, 0.3);
    eye1 = DrawEye(15*k,  G_sL7_LE, LinkColor2, 100, 0.3);

    %Draw cameras
    cy2 = DrawCylinder(5*k, 5*k, G_sL8_RE*[Ry(0), [ 0 0 15*k]'; 0 0 0 1], CamColor, 10, 1);
    cy2 = DrawCylinder(5*k, 5*k, G_sL8_LE*[Ry(0), [ 0 0 15*k]'; 0 0 0 1], CamColor, 10, 1);

    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%     axis('equal')
%     xlabel('x')
%     ylabel('y')
%     zlabel('z')
% 
% 
%     L1 = light;
%     set(L1, 'Position', [-20 -20 -2])
%     L2 = light;
%     set(L2, 'Position', [20 20 2])

end

T_Ro0 = G_sL0;
T_0I  = G_01*G_12*G_23*G_34*G_45*G_56 * G_6I;
