function [timeStamp occupiedTiles pitch yaw]=readHeadMotion3(fileName, W, H)

%% Head movement read
M = csvread(fileName,6,1);

timeStamp = M(:,1);
qx = M(:,2); qy = M(:,3); qz = M(:,4); qw = M(:,5); xf = M(:,6); yf = M(:,7); zf = M(:,8);

x = 2*qx.*qz+2*qy.*qw;
y = 2*qy.*qz-2*qx.*qw;
z = 1-2*qx.*qx-2*qy.*qy;

pitch = y;
yaw = x;

% tile maps
for ff = 1:length(pitch)
    occupiedTiles(ff,:) = tileMap(pitch(ff), yaw(ff), W, H);
%     image(100*reshape(occupiedTiles(ff,:),[H W]));
%     drawnow;    
end


end