function [Cx,DCx,DDCx,Zx,Cy,DCy,DDCy,Zy,Zx_ref]=KAJITA_MPC(N_step,Ts,N,Step_s,Com_h,Step_L,Step_W)

alpha=1e-6;
gamma=1;

g=9.8;

%% Matrices

A=[1 Ts (Ts^2)/2;
   0 1  Ts;
   0 0  1];

B=[(Ts^3)/6;
   (Ts^2)/2;
    Ts];

C=[1 0 -Com_h/g];

PS=[];
PZS=[];
 
for i=1:N
    PS=[PS;A^i];
    PZS=[PZS;C*(A^i)];
end

Pzs=PZS;

PU=[];
PZU=[];
 
ppurow=B;
pzurow=C*B;

for i=1:N
    
    if i>1
    ppurow=[ (A^(i-1))*B, ppurow];
    pzurow=[ C*(A^(i-1))*B, pzurow];
    end
    
    PU=[PU; ppurow, zeros(size(ppurow,1),N-i)];
    PZU=[PZU; pzurow, zeros(size(pzurow,1),N-i)];
end

Pzu=PZU;

Pps=[];
Pvs=[];
Ppu=[];
Pvu=[];
for i=1:3:3*N
Pps=[Pps;PS(i,:)];
Pvs=[Pvs;PS(i+1,:)];
Ppu=[Ppu;PU(i,:)];
Pvu=[Pvu;PU(i+1,:)];
end

Qt=alpha*eye(N)+gamma*Pzu'*Pzu;
Q=[Qt Qt*0;
   Qt*0 Qt]; 


xc=zeros(3,1);
Cx=[];
Zx=[];
 

%foot parameter for ZMP constraints
Sx=0.14; %Sole x dimension
Sy=0.07; %Sole y dimension

mx=0.01; %Safety margin x
my=0.01; %Safety margin y

Zx_ref=[0;0*ones(2*N,1)];
Zy_ref=[0;0*ones(2*N,1)];


for i=1:N_step
    if i>1
    Zx_ref=[Zx_ref;Zx_ref(end)+Step_L*ones(Step_s,1)];
    
    else
    Zx_ref=[Zx_ref;Zx_ref(end)+0*ones(Step_s,1)];
    end
    
    if mod(i,2)
        Zy_ref=[Zy_ref;Step_W*ones(Step_s,1)];
        
    else
        Zy_ref=[Zy_ref;-Step_W*ones(Step_s,1)];
    end
end


Zx_ref=[Zx_ref;Zx_ref(end)*ones(2*N,1)];
Zy_ref=[Zy_ref;0*ones(2*N,1)];

Tf_samples=size(Zx_ref,1);


Cx=zeros((Tf_samples-N),1);
Cy=zeros((Tf_samples-N),1);

DCx=zeros((Tf_samples-N),1);
DCy=zeros((Tf_samples-N),1);

DDCx=zeros((Tf_samples-N),1);
DDCy=zeros((Tf_samples-N),1);

Zx=zeros((Tf_samples-N),1);
Zy=zeros((Tf_samples-N),1);

yc=zeros(3,1);
index_i=0;

%constraints zmp belongs to foot sole
dx=[1;-1;0;0];
dy=[0;0;1;-1];

Dx=[];
Dy=[];
bk1=[];

for j=1:N
        Dx=blkdiag(Dx,dx);
        Dy=blkdiag(Dy,dy);
        bk1=[bk1;(Sx/2-mx);(Sx/2-mx);(Sy/2-my);(Sy/2-my)];
end

Dk1=[Dx,Dy];


opts = optimoptions('quadprog',...
    'Algorithm','interior-point-convex','Display','off');

%if we wanna add a disturbance dist=1
dist=0;

for i=1:(Tf_samples-N)
 
if i==round(Tf_samples/2) && dist
    xc(3)=xc(3)+.5;
end    
    
pkx=gamma*Pzu'*(Pzs*xc-Zx_ref(i:(i+N-1)));   
pky=gamma*Pzu'*(Pzs*yc-Zy_ref(i:(i+N-1)));

pk=[pkx;pky];

Ain=Dk1*[Pzu Pzu*0; Pzu*0 Pzu];

bin=bk1 - Dk1*[Pzs*xc;Pzs*yc]+Dk1*[Zx_ref(i:i+N-1);Zy_ref(i:i+N-1)];

uk=quadprog(Q,pk,Ain,bin,[],[],[],[],[],opts);

% Collect com and zmp data
Cx(i)=xc(1);
Cy(i)=yc(1);

DCx(i)=xc(2);
DCy(i)=yc(2);

DDCx(i)=xc(3);
DDCy(i)=yc(3);

% Zx(i)=C*xc;
ZZx=Pzs*xc+Pzu*uk(1:N);
Zx(i)=ZZx(1);

ZZy=Pzs*yc+Pzu*uk(N+1:end);
Zy(i)=ZZy(1);


%new state
xk1=A*xc+B*uk(1);
yk1=A*yc+B*uk(N+1);
%update state
xc=xk1;
yc=yk1;

end

index=1;

for i=(Tf_samples-N+1):Tf_samples
    %new state
xk1=A*xc+B*uk(index);
yk1=A*yc+B*uk(N+index);
%update state
xc=xk1;
yc=yk1;
index=index+1;

Cx=[Cx;xc(1)];
Cy=[Cy;yc(1)];

DCx=[DCx;xc(2)];
DCy=[DCy;yc(2)];

DDCx=[DDCx;xc(3)];
DDCy=[DDCy;yc(3)];

Zx=[Zx;C*xc];
Zy=[Zy;C*yc];


end

end

