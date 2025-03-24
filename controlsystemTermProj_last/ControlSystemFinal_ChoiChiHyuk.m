clear all
clc

% 시스템 변수 초기화
R  = 2.0;   % Ohms
L  = 0.5;   % Henrys
Km = 0.015; % Torque constant
Kb = 0.015; % EMF constant
Kf = 0.2;   % Nms
J  = 0.02;  % kg.m^2

% 상태 공간 행렬 정의
A = [-R/L -Kb/L; Km/J -Kf/J];
B = [1/L; 0];
C = [0 1];
D = [0];

% 상태 공간 모델 생성
sys_dc = ss(A, B, C, D);
sys_tf = tf(sys_dc);

% 성능 지표로부터 원하는 극점 계산
zeta    = 0.5;      % 감쇠비
tr      = 1.8;      % 상승 시간
omega_n = 1.8 / tr; % 자연 진동수

%% 5번
% 상태 공간 차원 계산
n = size(A, 1); % 상태 행렬의 행 수

% 제어 가능 행렬 계산
controllability_matrix = B;
for i = 1:n-1
    controllability_matrix = [controllability_matrix, A^i * B];
end

% 제어 가능성 확인
rank_C = rank(controllability_matrix); % 제어 가능 행렬의 랭크

if rank_C == n
    fprintf('The system is controllable.\n');
else
    fprintf('The system is NOT controllable.\n');
end

% 제어 가능 행렬과 랭크 출력
disp('Controllability Matrix:');
disp(controllability_matrix);
fprintf('Rank of Controllability Matrix: %d\n', rank_C);

t=0:0.01:10;
[y,x,t] = step(A,B,C,D,1,t);
info    = stepinfo(A,B,C,D)
plot(t,x)

desired_poles = roots([1 2 4]);
K = acker(A,B,desired_poles);

CHI = [A B; C D];
N   = inv(CHI)*[0;0;1];
Nx  = [13.3333; 1];
Nu  = [26.6817];
Nb  = Nu+K*Nx;

[num,den]    = ss2tf(A,B,C,D);
num_D        = [ 0 0 1.5];
den_D        = [1 -14 40.0225];
sys_G        = tf(num,den);
sys_D        = tf(num_D,den_D);
sys_symetric = sys_G * sys_D;
rlocus(sys_symetric)

row = 1450;
syms s
symetric_poly = 1+ row*(2.25)/(s^4 - 116*s^2 + 1602);
solve(symetric_poly);

desired_poles_syme = [-(2^(1/2)*(116 - 6002^(1/2)*1i)^(1/2))/2;-(2^(1/2)*(116 + 6002^(1/2)*1i)^(1/2))/2];

K_syme  = acker(A,B,desired_poles_syme);
Nb_syme = Nu + K_syme*Nx;

% 관측성 행렬 계산
O = obsv(A, C);

% 관측 가능성 확인
rank_O = rank(O);
n = size(A, 1); % 상태 변수의 수

if rank_O == n
    disp('The system is observable.');
else
    disp('The system is not observable.');
end

% 관측성 행렬 출력
disp('Observability Matrix:');
disp(O);

desired_error = 10*desired_poles;
L_esti = acker(A,B,desired_error);


desired_error_syme = 10*desired_poles_syme;
L_esti_syme        = acker(A,B,desired_error);

n = size(A, 1); % 기존 상태 개수
A_aug = [A, zeros(n, 1); -C, 0]; % 확장된 A 행렬
B_aug = [B; 0];                  % 확장된 B 행렬
C_aug = [C, 0];                  % 확장된 C 행렬
D_aug = D;


Int_desired_poles = [-zeta * omega_n + omega_n * sqrt(1 - zeta^2) * 1i, ...
                 -zeta * omega_n - omega_n * sqrt(1 - zeta^2), ...
                 -10]; % 추가된 적분 상태를 위한 극점

% 상태 피드백 게인 계산 (acker 함수 사용)
K_aug = acker(A_aug, B_aug, Int_desired_poles);

Kx_i = K_aug(1:n) % 기존 상태에 대한 피드백 게인
Ki   = K_aug(end) % 적분 상태에 대한 게인

Int_desired_poles_syme = [-(2^(1/2)*(116 - 6002^(1/2)*1i)^(1/2))/2;-(2^(1/2)*(116 + 6002^(1/2)*1i)^(1/2))/2; -10];

K_aug_syme = acker(A_aug, B_aug, Int_desired_poles_syme);

Kx_i_syme = K_aug_syme(1:n) % 기존 상태에 대한 피드백 게인
Ki_syme = K_aug_syme(end)   % 적분 상태에 대한 게인