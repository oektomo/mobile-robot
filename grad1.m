clc;
clear all;

qi=[ 3.9, 3.9];
qo=[ 4, 4];
ro=1.5;
deltax=[0.2, 0];
deltay=[0, 0.2];

norm((qi-qo),2)

f=1/(norm((qi-qo),2));
fdx=1/(norm((qi-qo+deltax),2));
fdy=1/(norm((qi-qo+deltay),2));

%f=1/(norm((qi-qo),2)-ro);
%fdx=1/(norm((qi-qo+deltax),2)-ro);
%fdy=1/(norm((qi-qo+deltay),2)-ro);

gradx = -(fdx-f)/0.2
grady = -(fdy-f)/0.2