function [ output_args ] = EPSStream( input_args )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
close all
directory = '/home/khan/Dropbox/Phd/MyPapers/iros2016/video/';
% pcdImagePath = '/home/mudassir/Dropbox/Phd/MyPapers/ICRA2016/video/groundTruthIllustrationPositiveObstacle.eps';
patch1 = imread(strcat(directory,'patch1.png'));
prmPatch1 = imread(strcat(directory,'prmPatch1.jpg'));
patch2 = imread(strcat(directory,'patch2.png'));
prmPatch2 = imread(strcat(directory,'prmPatch2.jpg'));

patch111 = imresize(patch1,[800 600]);
patch1 = imresize(patch1,800,600);
patch1 = resize(patch1,800,600);
patch1 = resize(patch1,800,600);
% 
% 
% F = [];
% fig = figure
% set(fig, 'Position', [0 0 1600 900])

v = VideoWriter('rti_safe_path.mp4');
v.FrameRate = 0.5;
open(v)
% for i=1:2
    
% img =image(patch1);
writeVideo(v,patch1);
writeVideo(v,prmPatch1);
%  F = [F;getframe(fig)];
%  img = image(patch2);
writeVideo(v,patch2);
writeVideo(v,prmPatch2);
% openfig(strcat(directory,'patch1.fig'))
%  F = [F;getframe(fig)];
% % image(prmPatch1);
% image(patch1);
%  F = [F;getframe(fig)];
% openfig(strcat(directory,'patch1.fig'))
%  F = [F;getframe(fig)];
% end
close(v)
% image(prmPatch1);
% movie(F,5,1)
% movie2avi(F,'rti_safe_path','fps',1)

end