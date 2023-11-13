clc
clear all

h = 590e3;
R = 6378.173e3;
mu = 3.986004418e14;
r = R + h;

n = sqrt(mu/(r^3));
T = 2*pi/n;

t_max = T/2;

n_imp = 2;

XF = [0 0 0 0 0 0]';
X0=[45 0 5 0 10 0 ]'; %condizioni [x0 y0 z0 vx0 vy0 vz0] prima del primo impulso

A = [zeros(3),eye(3);[3*(n^2) 0 0 0 2*n 0; 0 0 0 -2*n 0 0; 0 0 -n^2 0 0 0]];
phi = @(t)    expm(A*t);
X   = @(t,X0) phi(t)*X0;



HO=T/2; %metà tempo orbitale 
t=0;% tempo di impulso inizializzazione
DDV=1e15; %valore di inzializzazione molto grande per un delta V di inizializzazione

for i=1:HO

    posdes=XF(1:3);
posin=X0(1:3);
phiT=phi(i);
Vi=phiT(1:3,4:6)\(posdes-phiT(1:3,1:3)*posin); %velocità iniziali

%primo impulso ricavato dalla posizione finale che vogliamo avere
DV(1,i)= Vi(1)-X0(4);
DV(2,i)= Vi(2)-X0(5);
DV(3,i)= Vi(3)-X0(6);
%Calcolo velocità (in realtà anche la posizione che già conosciamo) final
%dopo l'impulso
Xi=[X0(1:3)',Vi']'; %posizioni iniziali subito dopo il primo impulso
Xf=X(i,Xi); %xf non desiderata
DV2=-Xf(4:6); %indifferente il segno per il risultato finale
I=sqrt(DV(1,i)^2+DV(2,i)^2+DV(3,i)^2)+sqrt(DV2(1)^2+DV2(2)^2+DV2(3)^2);
    if I<DDV % condizione di tempo inferiore
        DDV=I;
        t=i;
    end
   
end




   