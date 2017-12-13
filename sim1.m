clear all
clc
dh=0.2;
X=-5:dh:5;
Y=-5:dh:5;
[X,Y]=meshgrid(X,Y);
x0=[0;4];
xT=[0;-4];
Kr=30;
Vr=Kr./sqrt((X-x0(1)).^2+(Y-x0(2)).^2);
[fxr,fyr]=gradient(Vr,dh,dh);
Ka=25;
Va=.5.*Ka.*((X-xT(1)).^2+(Y-xT(2)).^2);
[fxa,fya]=gradient(Va,dh,dh);
cx1=1;
cy1=0;
cx2=-1;
cy2=0;
rad=2+.1;
th=(0:.1*dh:2).*pi;
xo=rad.*cos(th);
yo=rad.*sin(th);
xo1=xo-cx1;
yo1=yo-cy1;
xo2=xo-cx2;
yo2=yo-cy2;
Vo1=[];
Vo2=[];
Vo=zeros(size(Vr));
Ko=5;
for i=1:length(th)
Vo1=Ko./sqrt((X-xo1(i)).^2+(Y-yo1(i)).^2);
Vo2=Ko./sqrt((X-xo2(i)).^2+(Y-yo2(i)).^2);
Vo=Vo+Vo1+Vo2;
end
[fxo,fyo]=gradient(Vo,dh,dh);
fX=-fxr-fxa-fxo-fyo;
fY=-fyr-fya-fyo+fxo;
quiver(X,Y,fX,fY,'k')
title('Potential Field of Area')
xlabel('X-Axis')
ylabel('Y-Axis')
hold on
plot(x0(1),x0(2),'ro',xT(1),xT(2),'ro')
fill(xo1,yo1,'b',xo2,yo2,'b')
plot(xo1,yo1,'b',xo2,yo2,'b')
hold off
ss=1;
k=1;
xp=[];
yp=[];
xp(1)=x0(1);
yp(1)=x0(2);
ix=[];
iy=[];
jx=[];
jy=[];
fxx=0;
fyy=0;
while ss
Pw=sqrt(((X-xp(k)).^2)+((Y-yp(k)).^2));
xw(k)=min(min(Pw));
[iix,iiy]=find(Pw==xw(k));
ix(k)=iix(1);
iy(k)=iiy(1);
fx1=fX(ix(k),iy(k));
fy1=fY(ix(k),iy(k));
fxx(k)=fx1./norm(fX);
fyy(k)=fy1./norm(fY);
ff(k,:)=[fX(ix(k),iy(k)),fY(ix(k),iy(k))];
xp(k+1)=xp(k)+dh*(fxx(k));
yp(k+1)=yp(k)+dh*(fyy(k));
if (sqrt((xp(k+1)-xT(1)).^2+(yp(k+1)-xT(2)).^2)<=0.4)
ss=0;
end
end
k=k+1;
hold
plot([xp],[yp],'r')
figure
surf(X,Y,(Vr)+(Va)+(Vo))
title('Potential Surface')
