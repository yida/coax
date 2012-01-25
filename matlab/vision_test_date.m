function [] = vision_test_date(bagDir)

file = dir(strcat(bagDir,'/images/','*.jpg'));
nfile = size(file,1);

% set figure windows size
fig = figure;
set(fig,'visible','off');
%set(fig,'visible','off','Pos',[1,500,250,500],'PaperPositionMode','auto');
%set(fig,'Pos',[1,500,250,500],'PaperPositionMode','auto');
%set(fig,'PaperPositionMode','auto');

for fileIdx = 1 : nfile
    filename = file(fileIdx).name;
    imgRGB = imread(strcat(bagDir,'/images/',filename));    
    imgGray = rgb2gray(imgRGB);

    % Run Entropy test : whole
    disp(strcat('EP Test for whole-',num2str(filename),' image'));
    vision_EP(imgGray,fileIdx,'whole');
    % Run Entropy test : up
    disp(strcat('EP Test for upper-',num2str(filename),' image'));
    vision_EP(imgGray,fileIdx,'upper');
    % Run Entropy test : down
    disp(strcat('EP Test for lower-',num2str(filename),' image'));
    vision_EP(imgGray,fileIdx,'lower');
end
