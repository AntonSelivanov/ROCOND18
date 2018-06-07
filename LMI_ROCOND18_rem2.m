function flag=LMI_ROCOND18_rem2(A,B,C,K0bar,K1bar,h)
% This MATLAB program checks the feasibility of LMIs from Remark 2 of the paper 
% A. Selivanov and E. Fridman, "Improved sampled-data implementation of
% derivative-dependent control," IFAC Symposium on Robust Control Design, 2018. 

% The program uses YALMIP parser (http://users.isy.liu.se/johanl/yalmip/)

% Input: 
% A             - cell of matrices {A^1,...,A^M}
% B, C          - parameters of system (1)
% K0bar, K1bar  - nominal controller gains from (3) 
% h             - sampling period

% Output: 
% flag =1 if feasible, =0 otherwise

%% Common decision variables for all uncertainties 
[n,m]=size(B); 
P2=sdpvar(n,n,'f'); 
P3=sdpvar(n,n,'f'); 
W1=sdpvar(m); 
R1=sdpvar(m); 
%% LMIs 
LMIs=[]; 
if ~iscell(A)
    A={A}; 
end
for i=1:length(A)
    
    D=A{i}+B*K0bar*C+B*K1bar*C*A{i}; 
    %% Decision variables for current vertex A^i
    P=sdpvar(n); 
    W0=sdpvar(m); 
    
    Xi=blkvar; 
    Xi(1,1)=D'*P2+P2'*D; 
    Xi(1,2)=P-P2'+D'*P3; 
    Xi(1,3)=P2'*B; 
    Xi(1,4)=P2'*B; 
    Xi(1,5)=P2'*B; 
    Xi(2,2)=-P3-P3'+h^2*(K0bar*C)'*W0*(K0bar*C); 
    Xi(2,3)=P3'*B; 
    Xi(2,4)=P3'*B; 
    Xi(2,5)=P3'*B; 
    Xi(2,6)=h*(K1bar*C*A{i})'*(W1+R1/4); 
    Xi(3,3)=-pi^2/4*W0; 
    Xi(4,4)=-pi^2/4*W1; 
    Xi(5,5)=-R1; 
    Xi(6,6)=-(W1+R1/4); 
    Xi=sdpvar(Xi); 
    
    LMIs=[LMIs, P>=0, Xi<=0];  %#ok<AGROW>
end

%% Solution of LMIs
options=sdpsettings('solver','lmilab','verbose',0); 
sol=optimize(LMIs,[],options); 

flag=0; 
if sol.problem==0
    [primal,~]=check(LMIs); 
    flag=min(primal)>=0; 
else
    yalmiperror(sol.problem) 
end