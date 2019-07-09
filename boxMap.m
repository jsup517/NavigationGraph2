function occupiedTiles = boxMap(boundingBox, width, height, resX, resY)

% tile matching
%height = sqrt(T); width = sqrt(T);
occuTiles = zeros(height,width);

tileH = (boundingBox(2):1:boundingBox(4))/resY;
tileW = (boundingBox(1):1:boundingBox(3))/resX;
t1 = tileH*height;
t2 = tileW*width;

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