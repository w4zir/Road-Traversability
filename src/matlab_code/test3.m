d1 = 1.0;
dx = 0.5;
L = 2;
theta = 120*pi/180;
% alpha = 22*pi/180;


% d2*(cos(theta)-cos(theta+d2*tan(alpha)/L))/L - d1

syms d2 alpha;
[d2 alpha] = solve((L/tan(alpha))*(cos(theta)-cos(theta+d2*tan(alpha)/L)) == d1,(L/tan(alpha))*(sin(theta)-sin(theta+d2*tan(alpha)/L)) == dx);
%[d2 alpha] = solve(beta == d2*tan(alpha)/L, d2*(cos(theta)-cos(theta+beta))/beta == d1,d2*(sin(theta)-sin(theta+beta))/beta == dx);

% simplify(d2)