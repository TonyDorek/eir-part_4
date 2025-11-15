function u = u_nom_fun(kp, kd, xi, vi, xd)
%U_NOM_FUN Compute nominal acceleration controls (PD to goal)
%Inputs: agent position and velocity; Output: agent acceleration
u = -kp*(xi - xd) - kd*vi;
end

