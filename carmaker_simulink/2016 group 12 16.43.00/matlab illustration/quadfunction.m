function [a, b, r]=quadfunction(x,y)
% r^2=(x-a)^2 + (y-b)^2

x21 = x(2)-x(1); y21 = y(2)-y(1);
x31 = x(3)-x(1); y31 = y(3)-y(1);
h21 = x21^2+y21^2; h31 = x31^2+y31^2;
d = 2*(x21*y31-x31*y21);
a = x(1)+(h21*y31-h31*y21)/d;
b = y(1)-(h21*x31-h31*x21)/d;
r = sqrt(h21*h31*((x(3)-x(2))^2+(y(3)-y(2))^2))/abs(d);