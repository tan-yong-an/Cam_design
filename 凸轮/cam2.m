%  1. ��֪����
clear;
r0=89;   % ��Բ�뾶
rr=16;   % ���Ӱ뾶
h=89;   % �г�
e=59;   % ƫ��
delta01=120;  % �Ƴ��˶��ǣ��ȼ��ٵȼ���
delta02=30;   % Զ�ݽ�
delta03=60;   % �س��˶��ǣ������˶�
hd=pi/180;du=180/pi;
se=sqrt(r0*r0-e*e);
n1=delta01+delta02;
n3=delta01+delta02+delta03;

%  2. ͹���������
n=360
for i=1:n
    %---------------------  �����Ƹ��˶�����  ------------------
    if i<=delta01/2                                   % �Ƴ̽׶�
        s(i)=2*h*i^2/delta01^2;                        % �ȼ���
        ds(i)=4*h*i*hd/(delta01*hd)^2;ds=ds(i);
    elseif i>delta01/2 & i<=delta01                                 
        s(i)=h-2*h*(delta01-i)^2/delta01^2;              % �ȼ���
        ds(i)=4*h*(delta01-i)*hd/(delta01*hd)^2;ds=ds(i);
    elseif i>delta01 & i<=n1                           % Զ�ݽ׶�
        s(i)=h;ds=0;
    elseif i>n1 & i<=n3                               % �س̽׶�
        k=i-n1;
        s(i)=h-h*k/delta03;               % �����˶�
        ds(i)=-1*h/delta03;ds=ds(i);
    elseif  i>n3 & i<=n                              % ���ݽ׶�                               
        s(i)=0;;ds=0;
    end
    %-------------------  ����͹�ֹ켣����  ----------------------
    xx(i)=-(se+s(i))*sin(i*hd)-e*cos(i*hd);      % ����������������
    yy(i)=(se+s(i))*cos(i*hd)-e*sin(i*hd); 
    dx(i)=(ds-e)*sin(i*hd)+(se+s(i))*cos(i*hd); % ���㵼��
    dy(i)=(ds-e)*cos(i*hd)-(se+s(i))*sin(i*hd); 
    xp(i)=xx(i)+rr*dy(i)/sqrt(dx(i)^2+dy(i)^2)*(-1); % ����ʵ����������
    yp(i)=yy(i)-rr*dx(i)/sqrt(dx(i)^2+dy(i)^2); 
end

%  3. ���͹����������
figure(1);
hold on;grid on;axis equal;
axis([-(r0+h-30) (r0+h+10) -(r0+h+10) (r0+rr+10)]);
text(r0+h+3,4,'X');
text(3,r0+rr+3,'Y');
text(-6,4,'O');
title('ƫ��ֱ�������Ƹ�����͹�����');
xlabel('x / mm')
ylabel('y / mm')
plot([-(r0+h-40) (r0+h)],[0 0],'k');
plot([0 0],[-(r0+h) (r0+rr)],'k');
plot(xx,yy,'r-');                        % ��͹��������������
f1=[xx(:),yy(:)]; 
ct=linspace(0,2*pi);
plot(r0*cos(ct),r0*sin(ct),'g');            % ��͹�ֻ�Բ
plot(e*cos(ct),e*sin(ct),'c-');             % ��͹��ƫ��Բ
plot(e+rr*cos(ct),se+rr*sin(ct),'k');        % �����Բ
plot(e,se,'o');                         % ����Բ����
plot([e e],[se se+30],'k'); 
plot(xp,yp,'b');                        % ��͹��ʵ����������
f2=[xp(:),yp(:)];
