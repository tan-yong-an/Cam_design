%  1. 已知参数
clear;
r0=89;   % 基圆半径
rr=16;   % 滚子半径
h=89;   % 行程
e=59;   % 偏距
delta01=120;  % 推程运动角－等加速等减速
delta02=30;   % 远休角
delta03=60;   % 回程运动角－余弦运动
hd=pi/180;du=180/pi;
se=sqrt(r0*r0-e*e);
n1=delta01+delta02;
n3=delta01+delta02+delta03;

%  2. 凸轮曲线设计
n=360
for i=1:n
    %---------------------  计算推杆运动规律  ------------------
    if i<=delta01/2                                   % 推程阶段
        s(i)=2*h*i^2/delta01^2;                        % 等加速
        ds(i)=4*h*i*hd/(delta01*hd)^2;ds=ds(i);
    elseif i>delta01/2 & i<=delta01                                 
        s(i)=h-2*h*(delta01-i)^2/delta01^2;              % 等减速
        ds(i)=4*h*(delta01-i)*hd/(delta01*hd)^2;ds=ds(i);
    elseif i>delta01 & i<=n1                           % 远休阶段
        s(i)=h;ds=0;
    elseif i>n1 & i<=n3                               % 回程阶段
        k=i-n1;
        s(i)=h-h*k/delta03;               % 余弦运动
        ds(i)=-1*h/delta03;ds=ds(i);
    elseif  i>n3 & i<=n                              % 近休阶段                               
        s(i)=0;;ds=0;
    end
    %-------------------  计算凸轮轨迹曲线  ----------------------
    xx(i)=-(se+s(i))*sin(i*hd)-e*cos(i*hd);      % 计算理论轮廓曲线
    yy(i)=(se+s(i))*cos(i*hd)-e*sin(i*hd); 
    dx(i)=(ds-e)*sin(i*hd)+(se+s(i))*cos(i*hd); % 计算导数
    dy(i)=(ds-e)*cos(i*hd)-(se+s(i))*sin(i*hd); 
    xp(i)=xx(i)+rr*dy(i)/sqrt(dx(i)^2+dy(i)^2)*(-1); % 计算实际轮廓曲线
    yp(i)=yy(i)-rr*dx(i)/sqrt(dx(i)^2+dy(i)^2); 
end

%  3. 输出凸轮轮廓曲线
figure(1);
hold on;grid on;axis equal;
axis([-(r0+h-30) (r0+h+10) -(r0+h+10) (r0+rr+10)]);
text(r0+h+3,4,'X');
text(3,r0+rr+3,'Y');
text(-6,4,'O');
title('偏置直动滚子推杆盘形凸轮设计');
xlabel('x / mm')
ylabel('y / mm')
plot([-(r0+h-40) (r0+h)],[0 0],'k');
plot([0 0],[-(r0+h) (r0+rr)],'k');
plot(xx,yy,'r-');                        % 绘凸轮理论轮廓曲线
f1=[xx(:),yy(:)]; 
ct=linspace(0,2*pi);
plot(r0*cos(ct),r0*sin(ct),'g');            % 绘凸轮基圆
plot(e*cos(ct),e*sin(ct),'c-');             % 绘凸轮偏距圆
plot(e+rr*cos(ct),se+rr*sin(ct),'k');        % 绘滚子圆
plot(e,se,'o');                         % 滚子圆中心
plot([e e],[se se+30],'k'); 
plot(xp,yp,'b');                        % 绘凸轮实际轮廓曲线
f2=[xp(:),yp(:)];
