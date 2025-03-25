clear;
clc;
digits(6);%精度
HALFCK=60;%车宽的一半
L=140;%前轴与后轴的距离
TUJ=40;%凸轮与中心线的距离
Rtu=55;%基园
load xl.txt;
load yl.txt;
[a,b]=size(xl);
n=a*b;%取样点数量
TUI=1:1:n;%创建n维向量
zx=-1;%顺时针转向（逆时针取负）
for i=1:n
    if i==1%起始点
    x1=xl(n);    
    x2=xl(1);    
    x3=xl(2);
    y1=yl(n);    
    y2=yl(1);    
    y3=yl(2);
    if abs(((x1-x2)*(y2-y3))-((y1-y2)*(x2-x3)))<=1.00e-5
        TUI(1)=0;
    else
    z1=x2^2+y2^2-x1^2-y1^2;
    z2=x3^2+y3^2-x1^2-y1^2;
    z3=x3^2+y3^2-x2^2-y2^2;
    A=[(x2-x1),(y2-y1);(x3-x1),(y3-y1);(x3-x2),(y3-y2)];
    B=0.5*[z1;z2;z3];
    P0=(A'*A)\A'*B;
    R1=sqrt((P0(1)-xl(n))^2+(P0(2)-yl(n))^2 );
    R2=sqrt((P0(1)-xl(1))^2+(P0(2)-yl(1))^2 );
    R3=sqrt((P0(1)-xl(2))^2+(P0(2)-yl(2))^2 );
    R=(R1+R2+R3)/3;
    v=1;%当主动轮在左侧时为1，右侧为-1
    v1=[x1,y1]-[x2,y2];%当前点到前一点向量
    v2=[x3,y3]-[x2,y2];%当前点到后一点向量
    r=det([v1;v2]);                 %叉乘后第三个向量的方向
         if r>0    %凸轮在左侧第一个k取正（右侧取负）
            k=-1;      
         elseif r<0
            k=1;         
         end
    TUI(1)=vpa(zx*k*L*TUJ/(R+v*HALFCK));
    end
    elseif i==n%终点 
    x1=xl(n-1);   
    x2=xl(n);    
    x3=xl(1);
    y1=yl(n-1);    
    y2=yl(n);    
    y3=yl(1);
     if abs(((x1-x2)*(y2-y3))-((y1-y2)*(x2-x3)))<=1.00e-5
        TUI(n)=0;
    else
    z1=x2^2+y2^2-x1^2-y1^2;
    z2=x3^2+y3^2-x1^2-y1^2;
    z3=x3^2+y3^2-x2^2-y2^2;
    A=[(x2-x1),(y2-y1);(x3-x1),(y3-y1);(x3-x2),(y3-y2)];
    B=0.5*[z1;z2;z3];
    P0=(A'*A)\A'*B;
    R1=sqrt((P0(1)-xl(n-1))^2+(P0(2)-yl(n-1))^2 );
    R2=sqrt((P0(1)-xl(n))^2+(P0(2)-yl(n))^2 );
    R3=sqrt((P0(1)-xl(1))^2+(P0(2)-yl(1))^2 );
    R=(R1+R2+R3)/3;
    v=1;%当主动轮在左侧时为1，右侧为-1
    v1=[x1,y1]-[x2,y2];      %当前点到前一点向量
    v2=[x3,y3]-[x2,y2];      %当前点到后一点向量
    r=det([v1;v2]);                 %叉乘后第三个向量的方向
         if r>0    %凸轮在左侧第一个k取正（右侧取负）
            k=-1;      
         elseif r<0
            k=1;         
         end
    TUI(n)=vpa(zx*k*L*TUJ/(R+v*HALFCK));
     end
    else%其他点
    x1=xl(i-1);    
    x2=xl(i);    
    x3=xl(i+1);
    y1=yl(i-1);    
    y2=yl(i);    
    y3=yl(i+1);
     if abs(((x1-x2)*(y2-y3))-((y1-y2)*(x2-x3)))<=1.00e-5
        TUI(i)=0;
    else
    z1=x2^2+y2^2-x1^2-y1^2;
    z2=x3^2+y3^2-x1^2-y1^2;
    z3=x3^2+y3^2-x2^2-y2^2;
    A=[(x2-x1),(y2-y1);(x3-x1),(y3-y1);(x3-x2),(y3-y2)];
    B=0.5*[z1;z2;z3];
    P0=(A'*A)\A'*B;
    R1=sqrt((P0(1)-xl(i-1))^2+(P0(2)-yl(i-1))^2 );
    R2=sqrt((P0(1)-xl(i))^2+(P0(2)-yl(i))^2 );
    R3=sqrt((P0(1)-xl(i+1))^2+(P0(2)-yl(i+1))^2 );
    R=(R1+R2+R3)/3;%曲率半径
    v=1;%当主动轮在左侧时为1，右侧为-1    
    v1=[x1,y1]-[x2,y2];      %当前点到前一点向量
    v2=[x3,y3]-[x2,y2];      %当前点到后一点向量
    r=det([v1;v2]);                 %叉乘后第三个向量的方向
        if r>0    %凸轮在左侧第一个k取正（右侧取负）
           k=-1;      
        elseif r<0
           k=1;         
        end
         TUI(i)=vpa(zx*k*L*TUJ/(R+v*HALFCK));
     end
    end
end
for i=1:n%输出图像
    theta=i*2*pi/n;
    xx(i)=cos(theta)*(Rtu+TUI(i));
    yy(i)=sin(theta)*(Rtu+TUI(i));
    figure(1);
    plot(theta,Rtu+TUI(i),'k.');
    hold on;
    figure(2);        
    plot(xx(i),yy(i),'k.');
    hold on;
    theta2=0:0.1:2*pi;
    r=1;%滚子半径
    Circle1=xx(i)+r*cos(theta2);
    Circle2=yy(i)+r*sin(theta2);
    c1=Rtu*cos(theta2);
    c2=Rtu*sin(theta2);
    figure(3);
    plot(Circle1,Circle2,'c',c1,c2,'k');
    hold on;
end    
fid=fopen('tulun_cad.scr','w');%转化为CAD绘制脚本，运行程序后该程序所在文件夹会生成一个名为“tulun_cad.scr”的cad文件
fprintf(fid,'line\n');
for i=1:length(xx)
fprintf(fid,'%g,%g\n',xx(i),yy(i));
end
fclose(fid);
