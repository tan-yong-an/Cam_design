clear
clc

XYR=textread('guiji002.txt');
%% 小车参数

e=62.5;%基圆半径
m=-20; %前轮偏置距离  前轮中心若在小车中心左边则为负 右为正
L3=25;%实际摆杆与凸轮接触的X轴距离
L31=L3+2;% 加上凸轮厚度
L=172;%车长
M=185; %两个主动轮之间的距离
rr=2.5; % 摆杆半径

%% 计算区
 
X=XYR(:,2);%轨迹X坐标
Y=XYR(:,3);%轨迹Y坐标
R  =XYR(:,5);%曲率半径的数据
[length_guiji,~]=size(XYR); 
  
% 走的总距离
sum=0;
for i=2:length_guiji
    dt_sum=sqrt((X(i-1,1)-X(i,1))^2+(Y(i-1,1)-Y(i,1))^2);
    sum=sum+dt_sum;
end

% 计算相应轨迹长度
s=zeros(length_guiji,1);
s(1,1)=0;%路程赋初值
for i=2:length_guiji
    s(i,1)=s(i-1,1)+sqrt((X(i)-X(i-1))^2+(Y(i)-Y(i-1))^2)*(R(i,1)-M/2)/R(i,1);
    %计算小车走到第i点时主动轮走过路程
end

a1=zeros(length_guiji,1);
for i=1:length_guiji
    a1(i,1)=atan( L/ ( R(i,1)+m ) );
end
% 换算为角度
aa=rad2deg(a1);
 
% 计算角度
ct=zeros(length_guiji,1);
for j=1:length_guiji
    % ct(j,1)=2*pi*s(j,1)./s(length_guiji,1);
    ct(j,1)=2*295.3412/360*pi*s(j,1)./s(length_guiji,1);%开缺口
end
  

% 计算相应摆杆长度
E=zeros(length_guiji,1);
for i=1:length_guiji
    E(i,1)=L3/cos( a1(i,1) );
end
max_E=max(E);
 
%计算凸轮rou
rou=zeros(length_guiji,1);
rou1=zeros(length_guiji,1);
dt=zeros(length_guiji,1);
dt_shiji=zeros(length_guiji,1); % 由于摆杆本身厚度，故与理想dt有一定的偏差
 
for i=1:length_guiji
    dt(i,1)=E(i,1).*sin(a1(i,1));
    rou1(i,1)=dt(i,1)+e;
    dt_shiji(i,1)=dt(i,1)-rr*cos(a1(i,1));
    rou(i,1)=dt_shiji(i,1)+e;
end
 
max_rou=max(rou);
pianzhi=dt-dt_shiji;
baoluo=rou1-rou;
 
%% 绘画区 

figure(1)
hold on
axis equal;
title('轨迹打卡图');
x01=[5588,713;4463,375;2925,825;2175,1500;375,788;600,2550;375,2738;900,3938;1725,5250;3188,5625;];
rectangle('position',[-1000,-400,6000+1000,6000+700]);
plot(X,Y,'b-');
scatter(x01(:,1),x01(:,2),[],"r");
hold off
 
%绘画凸轮
figure(2)
subplot(1,2,1)
r00=zeros(length_guiji,1);
r00(:,1)=e;
polarplot(ct(:,1),rou(:,1));hold on;  %实际基圆
title('理想凸轮与实际凸轮对比图');
polarplot(ct,rou1);hold off;          %理想基圆
subplot(1,2,2)
polarplot(ct(:,1),rou(:,1));hold on;  %实际基圆
title('实际凸轮与基圆对比图');
polarplot(ct,r00,'--');hold off;      %基圆
 
hold off
 
%%  导出
 
for i=1:10000
   x(i)=rou(i)*cos(ct(i));
   y(i)=rou(i)*sin(ct(i));
   z(i)=0;
end
tulun=[x;y;z];
writematrix(tulun','tulun_new8.txt');
Z = zeros(10000, 1);
guiji=[X';Y';Z'];
writematrix(guiji','track.txt');