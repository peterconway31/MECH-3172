%% Honours Aircraft Preliminary Iterative Design
% Written by Peter Conway & Jake Williams
% a1703723
% The University of Adelaide
clear all
format short
sympref('FloatingPointOutput',true)
%% Fixed Variables
rho = 1.225; % [kg/m^3] density

syms TW; % thrust to weight ratio
syms WS; % wing loading
syms WP; % power loading

%% Mission Parameters
% Mission segment times [Hrs]

T_cruise = 1;
T_climb = 0.05;
T_loiter = 0.5;

T_vtol_loiter = 0.25;
T_vtol_climb = 0.03;
T_vtol_land = 0.02;

T_tot_conv =  T_cruise + T_climb +  T_loiter;
T_tot_vtol = T_vtol_loiter + T_vtol_climb + T_vtol_land;
T_tot = T_tot_conv + T_tot_vtol;

% speeds
V_cruise = 28; % m/s
V_stall = 18; %m/s
V_climb = 20; %m/s
V_loiter = 28; % m/s

% climb rates
RC_conv = 2; % m/s
RC_vtol = 3; % m/s
RC_vtol_land = -3; % m/s

% Power SF
SF_p = 1.4;
%% Initail Guess Values
% Weights
Mto = 7; % kg
M_prop = 0.6; % kg
M_payload = 1; % kg
MF_empty = 0.35; % [dimentionless]
Wto = Mto*9.81; % N
T = Wto;

% Layout
S = 0.7;
A = 0.5;
AR = 13.6; % 
e = 0.85; % 

% Battery paramers
e_lithium = 185;
F_use = 0.9;

% Effienciencies
eta_p = 0.8;
eta_p_hover = 0.7;

% Aero properties
Swet_S = 6; % preliminary Swet/S

Cl_max = 2.9; % with flaps and such

q = (1/2)*rho.*V_cruise.^2;

%% Drag 
% replace with polars
k = 1/(pi*AR*e);
cd_o = 0.01583;


%% Power Calcs
syms V_p alpha
% assume angle of attack remains const (alpha is climb angle)
Cl =  Wto/(0.5*S*rho*V_cruise^2);
Cd = cd_o + k*Cl^2;
LD = Cl/Cd;

L = 0.5*rho*V_p^2*S*Cl;
D = 0.5*rho*V_p^2*S*Cd;
Lx = L.*sin(alpha);
Ly = L.*cos(alpha);
Dx = D.*cos(alpha);
Dy = D.*sin(alpha);

V_x = V_p*cos(alpha);
V_y = V_p*sin(alpha);

%forward flight

Preq_x = (Dx + Lx).*V_x;
Preq_y = (Dy - Ly + Wto).*V_y;

Preq = sqrt(Preq_x^2 + Preq_y^2);

% hover + vertical hover climb
% for this config alpha = 90
MF = 1;

Preq_h = T*sqrt(T/A*1/(2*rho))/MF + V_p*D;

%% Mission segment power reqs

Pr_cruise = double(subs(Preq,{alpha,V_p},{0,V_cruise}));

climb_angle = asin(RC_conv/V_climb);
Pr_climb = subs(Preq,{alpha,V_p},{climb_angle,V_climb});

Pr_loiter = double(subs(Preq,{alpha,V_p},{0,V_loiter}));

Pr_vtol_loiter = double(subs(Preq_h,{V_p},{0}));

Pr_vtol_climb = double(subs(Preq_h,{V_p},{RC_vtol}));

Pr_vtol_land = double(subs(Preq_h,{V_p},{RC_vtol_land}));

% figure(1)
% title('Power Required Curve for hover')
% ezplot(Pr_vtol_climb)
% hold on
% xlim([0,2])
%ylim([0,15])
% 
%% Energy calcs
% segment energies
E_cruise = Pr_cruise*T_cruise;

E_climb = Pr_climb*T_climb;

E_loiter = Pr_loiter*T_loiter;

E_vtol_loiter = Pr_vtol_loiter*T_vtol_loiter;

E_vtol_climb = Pr_vtol_climb*T_vtol_climb;

E_vtol_land = Pr_vtol_land*T_vtol_land;

% mission profile total energies
E_conv = E_cruise + E_climb + E_loiter;

E_vtol = E_vtol_loiter +  E_vtol_climb + E_vtol_land;

% Battery Mass
M_batt = (1/(eta_p*F_use))*(E_conv/e_lithium) + (1/(eta_p_hover*F_use))*(E_vtol/e_lithium); 

%% Total mass iteration

Mto_i = (M_prop + M_payload + M_batt)/(1-(MF_empty));

Wto_i = Mto_i*9.81; % N


P_max_i = SF_p*subs(Preq_h,{V_p},{RC_vtol});
%% Iteration
%iteration starts here
N = 6; % number of iterations


for i = 1:N
    
    S_i = (Wto_i)/((1/2)*(V_stall^2)*rho*Cl_max);
    Cl_max =  Wto_i/(0.5*S*rho*V_stall^2);
    
    
    Cl =  Wto_i/(0.5*S_i*rho*V_cruise^2);
    Cd = cd_o + k*Cl^2;
    LD = Cl/Cd;

    L = 0.5*rho*V_p^2*S_i*Cl;
    D = 0.5*rho*V_p^2*S_i*Cd;
    Lx = L.*sin(alpha);
    Ly = L.*cos(alpha);
    Dx = D.*cos(alpha);
    Dy = D.*sin(alpha);

    V_x = V_p*cos(alpha);
    V_y = V_p*sin(alpha);

    %forward flight

    Preq_x = (Dx + Lx).*V_x;
    Preq_y = (Dy - Ly + Wto).*V_y;

    Preq = sqrt(Preq_x^2 + Preq_y^2);


    Preq_h = T*sqrt(T/A*1/(2*rho))/MF + V_p*D;
    
    Pr_cruise = subs(Preq,{alpha,V_p},{0,V_cruise});

    climb_angle = asin(RC_conv/V_climb);
    Pr_climb = subs(Preq,{alpha,V_p},{climb_angle,V_climb});

    Pr_loiter = subs(Preq,{alpha,V_p},{0,V_loiter});

    Pr_vtol_loiter = subs(Preq_h,{V_p},{0});

    Pr_vtol_climb = subs(Preq_h,{V_p},{RC_vtol});

    Pr_vtol_land = subs(Preq_h,{V_p},{RC_vtol_land});

    E_cruise = Pr_cruise*T_cruise;

    E_climb = Pr_climb*T_climb;

    E_loiter = Pr_loiter*T_loiter;

    E_vtol_loiter = Pr_vtol_loiter*T_vtol_loiter;

    E_vtol_climb = Pr_vtol_climb*T_vtol_climb;

    E_vtol_land = Pr_vtol_land*T_vtol_land;

    % mission profile total energies
    E_conv = E_cruise + E_climb + E_loiter;

    E_vtol = E_vtol_loiter +  E_vtol_climb + E_vtol_land;

    % Battery Mass
    M_batt_i = (1/(eta_p*F_use))*(E_conv/e_lithium) + (1/(eta_p_hover*F_use))*(E_vtol/e_lithium); 
    
    P_max_i = SF_p*subs(Preq_h,{V_p},{RC_vtol});
    
    M_prop_i = (0.256*P_max_i - 0.039)*10^-3;
    
    
    Mto_i = (M_prop_i + M_payload + M_batt_i)/(1-(MF_empty));

    Wto_i = Mto_i*9.81; % N
    
    i
    
    %i = i + 1;
end 


%% Thurst to Weight

V_RoC_conv = sqrt((2/rho)*WS*(sqrt((3*cd_o)/k)));

% Cruise Sizing
TW_cruise = q.*cd_o*(1/WS)+k*(1./q)*(WS);

% Stall Sizing 
WS_stall = vpa((1/2)*(V_stall^2)*rho*Cl_max);
% Conventional Loiter Sizing

% VTOl Loiter Sizing

% Conventional Climb Sizing

TW_conv_climb = (RC_conv/V_RoC_conv) + (1/(WS))*q*cd_o + (k./q)*(WS);

% VTOL Climb sizing

SF = 1.2; %20 extra T for manuveriing

TW_vtol_climb = SF*(1 + (1/WS)*rho*(RC_vtol^2)*(Swet_S));
Cd_VTOl = .07;
%S_vtol = S;
%TW_vtol_climb = SF*(1 + (1/WS)*0.5*rho*(RC_vtol^2)*(Cd_VTOl));


% Ceiling Sizing
TW_ceiling = (1/(2*V_RoC_conv))+(q./WS)*cd_o+(k./q)*WS;
%TW_ceiling = double(TW_ceiling);
WS_stall = 312.0237;
 % Landing Sizing


% figure(1)
% xlim([0,300])
% ylim([0,2])
% hold on
% 
% fplot(TW_cruise,'DisplayName','22 m/s Cruise','Linewidth',2) 
% fplot(TW_conv_climb,'DisplayName','2 m/s Climb','Linewidth',2) 
% fplot(TW_ceiling,'DisplayName','Service Ceiling','Linewidth',2)
% fplot(TW_vtol_climb,'DisplayName','VTOL Climb')
% xline(WS_stall(1),'DisplayName','12.5 m/s Stall Speed','Linewidth',2);
% legend('show','Location','best')
% title('Matching Diagram')
% ylabel('Thrust to Weight [T/W]')
% xlabel('Wing Loading [N/m^2]')


%% POWER LOADING 

% Cruise Sizing
PW_cruise = (TW_cruise.*V_cruise)/eta_p;

% Stall Sizing 

% Conventional Loiter Sizing

% VTOl Loiter Sizing

% Conventional Climb Sizing

%V_conv_climb = V_RoC_conv/sin(climb_angle);

%V_conv_climb = 26; % need to figure this out

PW_conv_climb = (TW_conv_climb*V_RoC_conv)/eta_p;


% VTOL Climb sizing

PW_vtol_climb = (TW_vtol_climb*RC_vtol)/eta_p_hover;
% Ceiling Sizing

PW_ceiling = (TW_ceiling*V_RoC_conv)/eta_p;
% Landing Sizing

%PW_cruise = (TW_cruise*V_cruise)/eta_p;


figure(2)
xlim([0,600])
ylim([0,15])
hold on

fplot(PW_cruise,'DisplayName','28 m/s Cruise','Linewidth',2) 
fplot(PW_conv_climb,'DisplayName','2 m/s Climb','Linewidth',2) 
fplot(PW_ceiling,'DisplayName','Service Ceiling','Linewidth',2)
fplot(PW_vtol_climb,'DisplayName','VTOL Climb')
xline(WS_stall,'DisplayName','12.5 m/s Stall Speed','Linewidth',2);
plot(312.0237,6.233,'ro')
text(320,6.5,num2str('P_1'),'FontSize',8)


legend('show','Location','best')
title('Matching Diagram')
ylabel('Power Loading [N/W]')
xlabel('Wing Loading [N/m^2]')





