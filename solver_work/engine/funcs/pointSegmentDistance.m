
% Input:
%   cx, cy; represents a point C
%   ax, ay, bx, by; represents a line segment AB
% Output:
%   xx yy; the point on segment AB closest to C
%   distance; the shortest distance from C to AB
%   nearVertex; 1 if the nearest point on the segment is a vertex
%               0 if the nearest point is on the segment
%
function [xx yy distance nearVertex] = pointSegmentDistance(cx, cy, ax, ay, bx, by)

% Test the unusal condition that (ax,ay) = (bx,by)
if ax==bx && ay==by
   xx = ax;   yy=ay;
   distance = sqrt((ax-cx)^2+(ay-cy)^2);
   return;
end

r_numerator = (cx-ax)*(bx-ax) + (cy-ay)*(by-ay);
r_denomenator = (bx-ax)*(bx-ax) + (by-ay)*(by-ay);
r = r_numerator / r_denomenator;

px = ax + r*(bx-ax);   py = ay + r*(by-ay);

s = ((ay-cy)*(bx-ax)-(ax-cx)*(by-ay)) / r_denomenator;
distanceLine = abs(s) * sqrt(r_denomenator);

% (xx,yy) is the point on the lineSegmnet closest to (cx,cy)
xx = px;   yy = py;

if r>=0 && r<=1
    nearVertex = 0;
    distance = distanceLine;
else
   nearVertex = 1;
   dist1 = (cx-ax)*(cx-ax) + (cy-ay)*(cy-ay);
   dist2 = (cx-bx)*(cx-bx) + (cy-by)*(cy-by);
   if dist1 < dist2
      xx = ax;   yy = ay;
      distance = sqrt(dist1);
   else
      xx = bx;   yy = by;
      distance = sqrt(dist2);
   end
end

