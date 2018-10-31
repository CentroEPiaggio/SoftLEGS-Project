function [ERRF,ERRC,Q]=solve_optimization(KAJ_COM,KAJ_FOOT,Ts,Nsolver)


global PFdes PCdes
% clc
ERRF=[];
ERRC=[];
Q=[];



Aeq=[1 1 1 0 0 0];
beq=0;
for ik=1:size(KAJ_COM,2)
PCdes=KAJ_COM(:,ik);
PFdes=KAJ_FOOT(:,ik);

lb=[-pi -pi -pi -pi -pi -pi];
ub=[pi 0 pi pi 0 pi];

% qp=q;
if ik==1
    qp=zeros(6,1); %it doesn't exists
    Ain=[];
    bin=[];
else
    qp=q;
    Ain=[eye(6);-eye(6)];
    bin=[ones(6,1)*4*Ts+qp;ones(6,1)*4*Ts-qp];
end


    
if ik==1
    flag_ok_first_post=1;
    count=0;
    while(flag_ok_first_post && count<=10)
        count=count+1;
        
opts = optimoptions(@fmincon,'Algorithm','interior-point','Display','off');

problem = createOptimProblem('fmincon','objective',...
 @min_err_pcpf,'x0',qp,'Aeq',Aeq,'beq',beq,'Aineq',Ain,'bineq',bin,'lb',lb,'ub',ub,'options',opts);
ms = MultiStart;
[q,f] = run(ms,problem,Nsolver);
 
[PFn,PCn]=DK_fun_fmincon(q);

errf=PFdes-PFn;
errc=PCdes-PCn;
qp=q;

if (max(abs(errf))>0.001) || (max(abs(errc(1:2)))>0.005) 
    flag_ok_first_post=1;
else
    flag_ok_first_post=0;
end

    end
end
    
opts = optimoptions(@fmincon,'Algorithm','sqp','Display','off');
problem = createOptimProblem('fmincon','objective',...
 @min_err_pcpf,'x0',qp,'Aeq',Aeq,'beq',beq,'Aineq',Ain,'bineq',bin,'lb',lb,'ub',ub,'options',opts);
ms = MultiStart;
[q,f] = run(ms,problem,Nsolver);

[PFn,PCn]=DK_fun_fmincon(q);

ERRF=[ERRF,PFdes-PFn];
ERRC=[ERRC,PCdes-PCn];
Q=[Q, q];    
    


% clc
end %end for KAJ
