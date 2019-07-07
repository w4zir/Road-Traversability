function [ col ] = get_colors_map( x )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

ran=range(x); %finding range of data
min_val=min(x);%finding maximum value of data
max_val=max(x)%finding minimum value of data
y=floor(((x-min_val)/ran)*63)+1; 
col=zeros(length(x),3)
p=colormap
for i=1:length(x)
  a=y(i);
  col(i,:)=p(a,:);
end

end

