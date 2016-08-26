clc
clear all
close all

colormap hsv
figure; hold on
angle = 41/180*pi;
alpha = 30/180*pi;
L = .2;
G = [0, 0, 0]
for radius = 0:.01:L
    theta = linspace(0,2*pi);
%     phi = linspace(0,pi/2);
    phi = linspace((pi-angle)/2,pi/2);
    [theta,phi] = meshgrid(theta,phi);
    [xs,ys,zs] = sph2cart(theta,phi,radius);
    mesh(xs+G(1),ys+G(2),zs+G(3));
%     mesh(xs+G(1)+.1,ys+G(2),zs+G(3));
%     mesh(xs,ys,zs);
    
%     mesh(xs+G(1),ys+G(2),zs+G(3));
end
colorbar
%     mesh(xs+G(1),ys+G(2),-zs+G(3))
    
hold off;
grid on;
xlabel('x(m)')
ylabel('y(m)')
zlabel('z(m)')
zlim([0 L]);