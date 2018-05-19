function [ s,t,w ] = generateGraph( rows, cols )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

nodes_count = rows*cols;

s = [[1:nodes_count] [1:nodes_count] [1:nodes_count]];
t = [[1:nodes_count]+cols-1 [1:nodes_count]+cols [1:nodes_count]+cols+1];
w = [sqrt(2)*ones(1,nodes_count) ones(1,nodes_count) sqrt(2)*ones(1,nodes_count)]

left_nodes = [1:cols:nodes_count];
right_nodes = [cols:cols:nodes_count];

idx = find(t>nodes_count);
s(idx) = [];
t(idx) = [];
w(idx) = [];

 s_left = (mod(s,cols)==1);
 s_right = (mod(s,cols)==0);
 t_left = (mod(t,cols)==1);
 t_right = (mod(t,cols)==0);
 
 st_1 = s_left.*t_right;
 st_2 = t_left.*s_right;
 
 idx1 = find(st_1==1);
 idx2 = find(st_2==1);
 idx = [idx1 idx2];
 
 s(idx) = [];
 t(idx) = [];
 w(idx) = [];
 
%  G = digraph(s,t);

end