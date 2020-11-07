%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                        TUNABLE PARAMETERS                               %
%              (ORDER, SETTLING TIME, ESO GAIN AND B0)                    %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
order = 2;                      % ADRC order
ts = 15;                        % Desired settling time
b0 = 1;                         % b0 parameter
eso_gain = 3;                   % ESO Dynamic related to closed-loop poles

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                 ADRC                                    %
%                  (CONTROLLER GAINS AND ESO MATRIXES)                    %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
s_cl = 4/ts;                    % Closed-loop system poles
s_eso = eso_gain * s_cl;        % ESO poles
pole = [1 s_cl];                % Base polynomial to calculate 
                                % System characteristic polynomial
pole_eso = [1 s_eso];           % Base polynomial to calculate 
                                % ESO characteristic polynomial
A = zeros(order + 1);           % Matrix A of ESO
B = zeros(order + 1, 1);        % Matrix B of ESO
C = [1 zeros(1, order)];        % Matrix C of ESO

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Here we'll calculate the characteristic polynomials of system and ESO,  %
% starting with the base polynomial, and convoluting it until we achieve  %
% the system's order.                                                     %
% Also, we'll put the values of A and B matrixes of ESO, filling the      %
% diagonal above the main diagonal of A with one's, and putting the value %
% b0 on the penultimate value of B matrix, according to ADRC generalized  %
% model fot Nth order.                                                    %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
A_sys = pole;                   
A_eso = pole_eso;

for i = 1:order
    if i ~= order
        A_sys = conv(A_sys, pole);
    else
        B(i) = b0;
    end
    A(i,i+1) = 1;
    A_eso = conv(A_eso, pole_eso);
end

K = [1 A_sys(2:end)];           % Controller gain matrix
L = A_eso(2:end)';              % ESO gain matrix