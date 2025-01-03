% =========================================================================
%% M.Sc. Laboratory Power Electronics - Experimental part II: Control of the ASM
% Based on REA
% 
% Datum: 23.01.2020
% =========================================================================
%
clc; % Clear Display
clear all
close all
disp('Informationen zur Simulation:')
basis =pwd;
p = [...
basis '\;'...
basis '\svm;'...
basis '\transformation;'...
];

path(p, path);


    plotcont            = 1;        % Plot analysis for time continuous design      (1/0)
    plotdisc            = 0;        % Plot analysis for discrete-time implementation (1/0)
    plot_comp_cont_disc = 1;        % Plot analysis for comparison time-continuous and time-discrete (1/0)
%% Grid parameters:
    f_N         = 50;               % Grid frequency [Hz]
    U_verk      = 400;              % Line-to-line voltage (rms) [V]

%% Inverter parameters
    f_sw        = 5e3;              % Switching frequency  5 kHz
    udc         = 400*sqrt(2);      % UDC charged from diode bridge (lab) ==> Uverk*sqrt(2)
    
    
    
    
%% Given machine parameters (for ASM in the star connection):
    ASM_p       = 1;                % Number of pole pairs
    ASM_Rs      = 3.9;             % Stator resistance [Ohm]
    ASM_Rr      = 1.6;             % Rotor resistance [Ohm], related to the primary side
    n_N         = 2895;              % Rated speed [min^-1]
    n_0         = f_N*60/ASM_p;     % Idle speed [min^-1] (Grid frequency[s^-1]*60[s/min]/p)
    P_N         = 2200;              % Rated shaft power [W]
    cos_phiN    = 0.85;              % Rated power factor
    ASM_J       = 0.0018;           % Moment of inertia [kg m^2]
    ASM_Lssigma = 0.00905;           % Stator leakage inductance [H]
    ASM_Lh      = 0.404;           % Main inductance       [H]
    ASM_Rh      = 442;             % Resistance parallel to main inductance  [Ohm]
    ASM_Lrsigma = 0.00905;           % Rotor leakage inductance  [H]
    I_S0        = 1.87;           % Idle current [A]
%
% Further machine parameters calculated from this:        
    ASM_Ls      = ASM_Lssigma + ASM_Lh;      % Stator inductance
    ASM_Lr      = ASM_Lrsigma + ASM_Lh;      % Rotor inductance
    ASM_sigma   = 1-ASM_Lh^2/(ASM_Ls*ASM_Lr);   % Magnetic leakage factor
    omega_N     = 2*pi*n_N/60*ASM_p;       % Nominal speed electric field
    sN          = 0.035;                        % Rated slip
    I_SN        = P_N/cos_phiN/U_verk/sqrt(3)*sqrt(2)*(cos(acos(cos_phiN))+1i*sin(acos(cos_phiN))); % Stator current
    Phase_I_SN  = angle(I_SN)*180/pi;       % Angle of the stator current
    S_Nenn      = P_N/cos_phiN ;            % Rated apparent power
    psi_RNcompl = ASM_Lrsigma*I_S0 + (ASM_Lh-ASM_Lrsigma)*I_SN ;  % Rotor flux: (REA)
    ASM_PSI_RN  = 0.98;             % Rated flux [Vs]

%% Limitations for simulation
    imax        = 6;                % Maximum reference current
    umax        = udc/sqrt(3);      % Maximum actuating voltage
    
% %Anti-Windup Match Factor x � [0.2...1]
%     x           = 1; %0.5;          % 0.03;
% %
% %###################################################                        
% %% Controller parameters    
% %###################################################     
% %
% Inner control loops
% Current controller (Technical optimum) ##################################################
    ASM_K          = (ASM_Lr^2)/(ASM_Rs*ASM_Lr^2+ASM_Rr*ASM_Lh^2);      % Proportional factor plant: ASM
    ASM_TN         = ASM_sigma*ASM_K*ASM_Ls;      % Time constant plant: ASM
    T_sw            =3/(2*f_sw)  ;     % Time constant of the inverter

    ASM_T1    = max(ASM_TN,T_sw);       % T1 corresponds to maximum time constant. 
                                         % This should be compensated
    ASM_Tsigma= min(ASM_TN,T_sw);       % Tsigma corresponds to minimum time constant

    % % PI controller % Implemented with GPI= KR + (KR/Tn)*1/s;
    CC_KP   = (ASM_T1)/(2*ASM_K*ASM_Tsigma) ;          % Technical optimum
    CC_KI   = CC_KP/ASM_T1;          % Technical optimum             
    CC_KAW  = 1/ASM_TN;          % Anti-Windup  z

% % #####################################################################
% %% Outer control loops
% %%% Flux controller (Technical optimum) ##################################################
     FC_K          = ASM_Lh;         % Gain factor of the plant of the flux 
     FC_TN         = ASM_Lr/ASM_Rr;         % Time constant flux
     T_approx_IC   = 2*T_sw;         % Approximated time constant inner control loop
 
     FC_T1              = max(FC_TN,T_approx_IC); % T1 corresponds to maximum time constant.                                                 
     FC_Tsigma          = min(FC_TN,T_approx_IC); % Tsigma corresponds to minimum time constant. 
 
     % PI Controller 
     FC_KP   = (FC_T1)/(2*FC_K*FC_Tsigma);     % Technical optimum 
     FC_KI   = FC_KP/FC_T1;     % Technical optimum  
     FC_KAW  = 1/FC_TN;     % Anti-Windup


%%% Speed controller (S0) ###############################################
    SC_TN             = ASM_J;      % Time constant: speed
    SC_K              = 3/2*ASM_p*(ASM_Lh/ASM_Lr)*ASM_PSI_RN;      %Gain factor of the speed plant

    SC_T1             = max(T_approx_IC,SC_TN);   % large time constant
    SC_T_sigma        = min(T_approx_IC,SC_TN);      %small time constant

    % PI Controller     % Symmetrical optimum 

    SC_a      = 10;                % Dynamic factor (Smooting factor)
    SC_KP     = SC_T1/(SC_a*SC_K*SC_T_sigma);    % Proportional factor of the speed controller
    SC_KI     = SC_KP/(SC_a^2*SC_T_sigma);    % Integral factor of the speed controller
    SC_KAW    = 1/(SC_a^2*SC_T_sigma);    % Anti-Windup  


% % #####################################################################
% % #################           Analysis                  ###############
% % #####################################################################
% %
%   Hints:   - No anti-windup structures or
% manipulated variable limits are considered!
%
%
%% Considerations in root locus and Bode (continuously)
if(plotcont==1)
s=tf('s');
% Plant transfer functions (All together)
G_PWM       = 1/(1+s*T_sw);         %PWM: (uS* -> uS: PWM)
G_ASM_lin   = ASM_K*(1/(1+ASM_TN*s));         %ASM: (uS -> iS: Stromdynamik ASM)
G_CC = 1/(1+s*T_sw*2);              %Current control loop: GL (3.26) (iS* -> iS: inner control loop)
G_FC   = FC_K*1/(1+s*FC_TN);            %Flux: GL (3.28) (isd -> PSI: Flux)
G_SC     = SC_K/(SC_TN*s);          %Speed: Gl (3.30)

%Inner loop
G_plant_CC  = G_PWM*G_ASM_lin;         %Plant current control
G_PI_CC     = CC_KP+(CC_KI/s);         %PI  controller current control
G_CC_OL     = G_plant_CC*G_PI_CC;         %Open current control loop
G_CC_CL     = G_CC_OL/(1+G_CC_OL);         %Closed current control loop


% disp('Step response current control loop (cont):')
% stepinfo(G_CC_CL)

% figure ('Name','Current control loop (cont)')
%     subplot(2,2,1)
%         bode(G_CC_OL)        %Blue: Plant, Green: Plant+Controller
%         margin(G_CC_OL)                 
%         grid on
%     subplot(2,2,2)
%         rlocus(G_CC_OL)                 
%         grid on
%     subplot(2,2,3)
%         step(G_CC_CL)
%         grid on
%     subplot(2,2,4)
%         pzmap(G_CC_OL);
%     %pzmap(G_PI_CC,G_plant_CC)   
%         grid on

% Outer control loops

% Flux control loop
G_plant_FC  = G_CC*G_FC;                        % Plant flux control loop
G_PI_FC     = FC_KP + FC_KI/s;                  % PI controller flux control loop
G_FC_OL     = G_PI_FC*G_plant_FC;               % Open flux control loop
G_FC_CL     = feedback(G_FC_OL,1);              % Closed flux control loop

% figure ('Name','Flussregelkreis (cont)')
%     subplot(2,2,1)
%         bode(G_plant_FC,G_FC_OL)
%         grid on
%     subplot(2,2,2)
%         rlocus(G_FC_OL)
%         grid on
%     subplot(2,2,3)
%         step(G_FC_CL)
%         grid on
%     subplot(2,2,4)
%         pzmap(G_PI_FC,G_plant_FC)
%         grid on

 % Speed control loop
G_plant_SC  = G_CC*G_SC;                         % Plant speed control loop
G_PI_SC     = SC_KP + SC_KI/s;                   % PI controller speed control loop
G_SC_OL     = G_PI_SC*G_plant_SC;                % Open speed control loop
G_SC_CL     = feedback(G_SC_OL,1);             % Closed speed control loop

% figure ('Name','Speed control loop (cont)')
%     subplot(2,2,1)
%         bode(G_plant_SC,G_SC_OL)
%         grid on
%     subplot(2,2,2)
%         rlocus(G_SC_OL)
%         grid on
%     subplot(2,2,3)
%         step(G_SC_CL)
%         grid on
%     subplot(2,2,4)
%         pzmap(G_PI_SC,G_plant_SC)
%         grid on
end
% #####################################################################
% #################            Time discrete            ###############
% #####################################################################
%% Considerations in (time) discrete
fa      =   f_sw;        % Sampling frequency = switching frequency
Ta      =   1/fa;        % Sampling period duration

z=tf("z",Ta);

%Euler-forward
G_PWM_zoh              = c2d(G_PWM,Ta,'zoh'); % PWM delay
G_ASM_lin_zoh        = c2d(G_ASM_lin,Ta,'zoh'); 
G_CC_zoh      = c2d(G_CC,Ta,'zoh'); 
G_FC_zoh        = c2d(G_FC,Ta,'zoh'); 
G_SC_zoh          = c2d(G_SC,Ta,'zoh'); 

%Inner loop
G_plant_CC_zoh  = G_PWM_zoh*G_ASM_lin_zoh;         %Plant current control
G_PI_CC_zoh     = CC_KP+(CC_KI/(z-1)*Ta);         %PI  controller current control
G_CC_OL_zoh     = G_plant_CC_zoh*G_PI_CC_zoh;         %Open current control loop
G_CC_CL_zoh     = G_CC_OL_zoh/(1+G_CC_OL_zoh);         %Closed current control loop

%Euler-backward
G_PWM_foh              = c2d(G_PWM,Ta,'foh'); % PWM delay
G_ASM_lin_foh        = c2d(G_ASM_lin,Ta,'foh'); 
G_CC_foh      = c2d(G_CC,Ta,'foh'); 
G_FC_foh        = c2d(G_FC,Ta,'foh'); 
G_SC_foh          = c2d(G_SC,Ta,'foh'); 

%Inner loop
G_plant_CC_foh  = G_PWM_foh*G_ASM_lin_foh;         %Plant current control
G_PI_CC_foh     = CC_KP+(CC_KI/(z-1)*z*Ta);         %PI  controller current control
G_CC_OL_foh     = G_plant_CC_foh*G_PI_CC_foh;         %Open current control loop
G_CC_CL_foh     = G_CC_OL_foh/(1+G_CC_OL_foh);         %Closed current control loop

disp('Discretation')
stepinfo(G_CC_CL)

% figure ('Name','Discretation for Current control loop (cont)')
%         bode(G_CC_OL)        %continuous
%         margin(G_CC_OL)                 
%         grid on
%         hold;
%         bode(G_CC_OL_zoh)        %forward
%         margin(G_CC_OL_zoh) 
%         hold;
%         bode(G_CC_OL_foh)        %backward
%         margin(G_CC_OL_foh) 
%         grid on
%     subplot(2,2,4)   &tustin
%         pzmap(G_CC_OL);
%     %pzmap(G_PI_CC,G_plant_CC)   
%         grid on


%sim('asm_basis_translated')


clear mex
% disp('mex Files...')
% cd ([basis,'\svm\'])
mex s_svm.c svm.c
% cd ([basis,'\transformation\'])
mex s_dq_svm_transformation.c 
mex s_innerloop.c innerloop.c 
% cd ..
% cd (basis)

