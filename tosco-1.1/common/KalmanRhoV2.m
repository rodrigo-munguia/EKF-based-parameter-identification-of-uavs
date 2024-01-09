function [rho_new Prho_new alfa_new Palfa_New] = KalmanRhoV2(rho, Prho,alfa,Palfa, d,al, U, R)

   x = [rho alfa]';
   P = diag([Prho Palfa]);

% Prediction Equations
    
    
    %x = (1-(lambda_w*delta_t))*x
    P = P + U;
    
    % Update Equations
    
    S = P + R;
   % K =  P*inv(S);
    K = P/S;   
    
    z = [d al]';
    
    x = x + K*(z-x);   
    
    %P = (1-K)*P;    % Opcion 1
    P =  P - K*S*K'; % Opcion 2
    %ensure P remains symmetric
    P = 0.5*(P+P');
    
    
   rho_new = x(1);
   alfa_new = x(2);
   Prho_new = P(1,1);
   Palfa_New = P(2,2);