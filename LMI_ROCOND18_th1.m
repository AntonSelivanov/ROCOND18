function flag=LMI_ROCOND18_th1(A,B,C,K0bar,K1bar,h)
% This MATLAB program checks the feasibility of LMIs from Theorem 1 of the paper 
% A. Selivanov and E. Fridman, "Improved sampled-data implementation of
% derivative-dependent control," IFAC Symposium on Robust Control Design, 2018. 

% The program uses YALMIP parser (http://users.isy.liu.se/johanl/yalmip/)

% Input: 
% A, B, C       - parameters of system (1)
% K0bar, K1bar  - nominal controller gains from (3) 
% h             - sampling period

% Output: 
% flag =1 if feasible, =0 otherwise

%% Decision variables
[n,m]=size(B);
P=sdpvar(n); 
P2=sdpvar(n,n,'f'); 
P3=sdpvar(n,n,'f'); 
W0=sdpvar(m); 
W1=sdpvar(m); 
R1=sdpvar(m); 
%% Notations 
D=A+B*K0bar*C+B*K1bar*C*A; 
%% LMIs 
N=blkvar; 
N(1,1)=D'*P2+P2'*D; 
N(1,2)=P-P2'+D'*P3; 
N(1,3)=P2'*B; 
N(1,4)=P2'*B; 
N(1,5)=P2'*B; 
N(2,2)=-P3-P3'+h^2*(K0bar*C)'*W0*(K0bar*C)+h^2*(K1bar*C*A)'*(W1+R1/4)*(K1bar*C*A); 
N(2,3)=P3'*B; 
N(2,4)=P3'*B; 
N(2,5)=P3'*B; 
N(3,3)=-pi^2/4*W0; 
N(4,4)=-pi^2/4*W1; 
N(5,5)=-R1; 
N=sdpvar(N); 

%% Solution of LMIs
LMIs=[P>=0, N<=0]; 
options=sdpsettings('solver','lmilab','verbose',0); 
sol=optimize(LMIs,[],options); 

flag=0; 
if sol.problem==0
    [primal,~]=check(LMIs); 
    flag=min(primal)>0; 
else
    yalmiperror(sol.problem) 
end