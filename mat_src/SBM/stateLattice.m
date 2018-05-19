function [ output_args ] = stateLattice( input_args )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
% close all

L = 2;
p1 = [0.08 0 90];
beta = 15;
max_phi = 40;

plot(p1(1), p1(2),'k*');
hold on
axis([-5 5 -5 5])

for alpha=-max_phi:10:max_phi
    Rn = L/tand(alpha);
    d = 2*beta*Rn*pi/180;
    
    p = [];
    for x = 1:10
        dn = x*d/10;
        beta_n = sign(alpha)*x*beta/10;
        %         p0 = [x;0;90];
        xn = p1(1) + (sind(p1(3)+beta_n) - sind(p1(3)))*Rn;
        yn = p1(2) + (cosd(p1(3)) - cosd(p1(3)+beta_n))*Rn;
        tn =  p1(3) + beta_n;
        
        pn = [xn yn tn]
        p = [p; pn];
    end
    plot(p(:,1),p(:,2),'b-')
end


%  plot(p(:,1),p(:,2),'b-')

end

