function occupiedTiles = tileMap(pitch, yaw, width, height)

% tile matching
%height = sqrt(T); width = sqrt(T);
occuTiles = zeros(height,width);

viewportH = 0.14;
viewportW = 0.14;
tileH = pitch-viewportH:0.01:pitch+viewportH;
tileW = yaw-viewportW:0.01:yaw+viewportW;
t1 = (tileH+1)/2*height;
t2 = (tileW+1)/2*width;

tileHmin = floor(min(t1)); tileHmax = ceil(max(t1));
tileWmin = floor(min(t2)); tileWmax = ceil(max(t2));

occuH = tileHmin:1:tileHmax;
occuW = tileWmin:1:tileWmax;

occuH = (occuH>=0& occuH<height).*occuH + (occuH<0).*(occuH+height) + (occuH>=height).*(occuH-height);
occuW = (occuW>=0 & occuW<width).*occuW + (occuW<0).*(occuW+width) + (occuW>=width).*(occuW-width);

for hh=1:length(occuH)
    for ww=1:length(occuW)
        occuTiles(occuH(hh)+1,occuW(ww)+1) = 1;
    end
end

occupiedTiles = reshape(occuTiles,[1 width*height]);

end