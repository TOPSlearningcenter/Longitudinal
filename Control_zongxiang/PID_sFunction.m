function [sys,x0,str,ts]=PID_sFunction(t,x,u,flag)
switch flag
    case 0
        [sys,x0,str,ts]=mdlInitializeSizes;
    case 2
        sys=mdlUpdate(t,x,u);
    case 3
        sys=mdlOutputs(t,x,u);
    case {1, 4, 9}
        sys = [];
    otherwise
        error(['Unhandled flag = ',num2str(flag)]);
end
end

function [sys,x0,str,ts]=mdlInitializeSizes
sizes = simsizes;
sizes.NumContStates  = 0;
sizes.NumDiscStates  = 3;
sizes.NumOutputs     = 4;
sizes.NumInputs      = 1;
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 1;
sys=simsizes(sizes);
x0=[0,0,0]';
str=[];
ts=[0.01 0]; % 采样时间 偏移量


% 初始处理化处理计算整条轨迹的车速
load("reference.mat")

% 算参考车速
v_ref=zeros(size(t_ref));

for i=1:length(x_ref)-1
    Delta_dis(i) = norm([x_ref(i+1)-x_ref(i),y_ref(i+1)-y_ref(i)]);
end
v_ref(1:end-1) = Delta_dis./diff(t_ref);
v_ref(end)=v_ref(end-1);

save("reference_processed.mat",'t_ref','x_ref',"y_ref","v_ref");

end

function sys=mdlUpdate(t,x,u)

ts=0.01;
load("reference_processed.mat")

v_ref1=interp1(t_ref,v_ref,t);

sys(2)=x(1);
sys(1)=v_ref1-u(1);
sys(3)=x(3)+ts*x(1);

% sys(1) v_err_k 本时刻车速差
% sys(2) v_err_k-1 上一时刻车速差
% sys(3) v_err_int 车速误差积分


end

function sys=mdlOutputs(t,x,u)

ts=0.01;

kp=15;
ki=3;
kd=0.1;

ax=kp*x(1)+ki*x(3)+kd*(x(1)-x(2))/ts;


m=2000; 
a=1.4;
b=1.6;
L=a+b;
g=9.8;

Fz_f=m*g*b/L;
Fz_r=m*g*a/L;
Fx=m*ax;
% 按照垂向载荷分配纵向力
Fx_f = Fx*Fz_r/(m*g);
Fx_r = Fx*Fz_f/(m*g);


sys(1)=0.5.*Fx_f;
sys(2)=0.5.*Fx_f;
sys(3)=0.5.*Fx_r;
sys(4)=0.5.*Fx_r;

end
