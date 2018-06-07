% This MATLAB program checks the feasibility of LMIs from Theorem 1 and Remark 2 of the paper 
% A. Selivanov and E. Fridman, "Improved sampled-data implementation of
% derivative-dependent control," IFAC Symposium on Robust Control Design, 2018. 

%% Example 1 [Liu and Fridman (2012)]
% System parameters 
A={[0 1; -.1 0],[0 1; .1 0]}; % Uncertain system matrix
B=[0; 1]; C=[1 0]; 

K0bar=-.25; K1bar=-.0499;   % Nominal controller gains of (3)
h=.258;                     % Sampling period 

if LMI_ROCOND18_rem2(A,B,C,K0bar,K1bar,h)    
    disp('Example 1: LMIs are feasible');     
    % Calculating controller gains of (7) using (6) 
    disp(['K0=' num2str(K0bar+K1bar/h)]); 
    disp(['K1=' num2str(-K1bar/h)]); 
else 
    disp('Example 1: LMIs are not feasible'); 
end
%% Example 2 [Ortega-Montiel et al. (2017) - Furuta pendulum]
% System parameters 
A=[0 1 0 0; 37.377 -.515 0 .142; 0 0 0 1; -8.228 .113 0 -.173]; 
B=[0; -35.42; 0; 43.28]; 
C=[1 0 0 0; 0 0 1 0]; 

% Calculating nominal controller gains of (3)
Kplace=-place(A,B,[-1, -1.1, -1.2, -1.3]); 
K0bar=Kplace([1 3]); 
K1bar=Kplace([2 4]); 

h=.103; % Sampling period 

if LMI_ROCOND18_th1(A,B,C,K0bar,K1bar,h)
    disp('Example 2: LMIs are feasible'); 
    % Calculating controller gains of (7) using (6) 
    disp(['K0=[' num2str(K0bar+K1bar/h) ']']); 
    disp(['K1=[' num2str(-K1bar/h) ']']); 
else 
    disp('Example 2: LMIs are not feasible'); 
end