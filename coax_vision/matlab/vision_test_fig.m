function [] = vision_test_fig(bagDir)

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

    % Run bright pixel test : median whole
    disp(strcat('BP Test for whole-',num2str(filename),' image with median'));
    vision_BP(imgGray,fileIdx,'median','whole');
    % Run bright pixel test : median up
    disp(strcat('BP Test for upper-',num2str(filename),' image with median'));
    vision_BP(imgGray,fileIdx,'median','upper');
    % Run bright pixel test : median down
    disp(strcat('BP Test for lower-',num2str(filename),' image with median'));
    vision_BP(imgGray,fileIdx,'median','lower');
    
end
