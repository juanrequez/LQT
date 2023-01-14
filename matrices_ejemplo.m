%Matriz A
A=[-2.0500   -0.4000         0         0         0         0
    0.2500         0         0         0         0         0
         0         0   -2.2000   -0.8000         0         0
         0         0    0.5000         0         0         0
         0         0         0         0   -2.2500   -0.5000
         0         0         0         0    1.0000         0];
%Matriz B    
B=[1.0000         0
         0         0
         0    0.5000
         0         0
         0    0.5000
         0         0];
%Matriz C
C= [0.0719   -0.5752   -0.1594    0.6378         0         0
    0.0959   -0.7668         0         0    0.1664   -0.3329];    

%matriz Q2 %estos valores pueden elegirse (semidefinida positiva)
Q2=40*eye(2);
%Matriz R (pueden elegirse, definida positiva)
R=eye(2);
%Matriz L
L=C'*inv(C*C');
%Matriz Q (ya no puede elegirse sus valores)
Q=C'*Q2*C;

%Resolver la ecuación algebraica de Riccati 
[K,P,E]=lqr(A,B,Q,R)
%Resolver la ganancia de adelanto
Kff=inv(R)*B'*inv((A-B*K)')*Q*L;

%Controlador previo
%Matriz Q
Qp=eye(6);
%Matriz R
Rp=eye(2);
%Resolver la ecuación algebraica de Riccati 
[Kp,Pp,E]=lqr(A,B,Qp,Rp)