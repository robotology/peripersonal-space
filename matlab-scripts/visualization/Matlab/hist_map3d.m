%% About 
% Function to display the receptive field of a skin taxel
% pos: position of skin taxel
% range: vector of sample of distance to taxel
% parzenFunc: parzen estimation value of pps activation
% varargin(1): resolution of visualization
% varargin(2): transparent of visualization

% Author: NGUYEN Dong Hai Phuong
% Email: phuong.nguyen@iit.it; ph17dn@gmail.com

%%
function [ h ] = hist_map3d( pos, range, parzenFunc,varargin )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

% figure; hold on
% colormap hsv

res = 100;
transparent = 1;

if (~isempty(varargin))
    if (length(varargin)>=1)
        res = varargin{1};
    elseif(length(varargin)>=2)
            res = varargin{1};
            transparent = varargin{2};
    end
end

angle = 80/180*pi;  % cone angle
L = range(end)-range(1);
% L = radius;
% L = 0.2;
G = pos;
% l_p = length(parzenFunc)
% p = parzenFunc

d = L/length(parzenFunc);
a = repmat(parzenFunc',[1,length(parzenFunc)]);
size(a);
i = 1;
% for radius = 0:10*d:L-d
for radius = range(1):10*d:range(end)
%     iValue = i
%     rad = radius
    theta = linspace(0,2*pi,res);
    
%     phi = linspace(0,pi/2);
    phi = linspace((pi-angle)/2,pi/2,res);
    [theta,phi] = meshgrid(theta,phi);
    [xs,ys,zs] = sph2cart(theta,phi,radius);
%     surf(xs+G(1),ys+G(2),zs+G(3),gradient(zs+G(3)));
%     mesh(xs,ys,zs);
%     parzen = parzenFunc(10*i)
    b = parzenFunc(10*i)*ones(length(theta));
    h(i) = mesh(xs+G(1),ys+G(2),zs+G(3),b);
    set(h(i),'FaceAlpha',transparent);
    i = i+1;
    if i>(length(parzenFunc)/10)
        break;
    end
end
colorbar
%     mesh(xs+G(1),ys+G(2),-zs+G(3))
    
% hold off;
% grid on;
% xlabel('x(m)')
% ylabel('y(m)')
% zlabel('z(m)')
% zlim([range(1)-0.01 range(end)+0.01]);

end

