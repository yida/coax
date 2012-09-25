
%linearImg = linear_cam();
k = 1;
dt = 0.133;
dir = zeros(size(linearImg));

for Row = 1:size(linearImg,1)-1
refI = round(linearImg(Row,:));
nextI = round(linearImg(Row+1,:));
[~,refIm] = size(refI);

%out = zeros(3,refIm);

rightShift = circshift(refI,[0,k]);
%rightShift(1,1:k) = 0;
leftShift = circshift(refI,[0,-k]);
%leftShift(1,refIm-k+1:refIm) = 0;
 
shiftSum = sum((nextI-refI).*(leftShift-rightShift));
disSum = sum((leftShift-rightShift).^2);
s = 2*k*shiftSum/disSum;

Ihat = refI + (leftShift-rightShift)*s/(2*k);
Ihat = round(Ihat);

dir(Row,:) = Ihat - refI;

%{
out(1,:) = Ihat; 
out(2,:) = refI;
out(3,:) = nextI;
%}
%out = uint8(out);

%imshow(uint8(out));
end;
imagesc(linearImg);
colormap(gray);
hold on;
quiver(1:refIm,1:size(linearImg,1),dir,zeros(size(dir)));
%axis([1 64 0 22]);
axis equal;
hold off;