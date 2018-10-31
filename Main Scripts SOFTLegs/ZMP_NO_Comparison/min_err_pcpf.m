function err=min_err_pcpf(q)
[PF,PC] = DK_fun_fmincon(q);
global PFdes PCdes

err=norm(PF-PFdes,2)+norm(PC(1:2,:)-PCdes(1:2,:),2);
end


