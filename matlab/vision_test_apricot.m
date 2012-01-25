function [] = vision_test_apricot(bagDir)

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

    % Run bright pixel test : mean whole
    disp(strcat('BP Test for whole-',num2str(filename),' image with mean'));
    vision_BP(imgGray,fileIdx,'mean','whole');
    % Run bright pixel test : mean up
    disp(strcat('BP Test for upper-',num2str(filename),' image with mean'));
    vision_BP(imgGray,fileIdx,'mean','upper');
    % Run bright pixel test : mean down
    disp(strcat('BP Test for lower-',num2str(filename),' image with mean'));
    vision_BP(imgGray,fileIdx,'mean','lower');
end
