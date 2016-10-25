function models = compute_tf_models(P)
% Compute the simplified linear transfer function models that will be used
% to analytically develop autopilot control PID gains.
%
%   models = compute_tf_models(P)
%
%   Inputs:
%      P:       uavsim paramter structure
%
%   Outputs:
%      models:  Structure containing resulting simplified tranfer function
%               models, as well as coefficients used to create the TF
%               models.  (Having access to the coefficients will be useful
%               in developing the autopilot gains.)
%
% Developed for JHU EP 525.461, UAV Systems & Control
% Adapted from design project in "Small Unmanned Aircraft: Theory and
% Practice", RWBeard & TWMcClain, Princeton Univ. Press, 2012
%

% Define Laplace s
s=tf('s');


rho = P.rho;
Va2 = P.Va0*P.Va0;
Va0 = P.Va0;

%
% Aileron channel coefficients and models
%

bigGamma = P.Jx*P.Jz-P.Jxz*P.Jxz;
S = P.S_wing;
models.a_phi1 = -(rho*Va2*S*P.b/2) * ((P.Jz*P.C_ell_p+P.Jxz*P.C_n_p)/bigGamma) * (P.b/2/Va0);
models.a_phi2 = (rho*Va2*S*P.b/2) * ((P.Jz*P.C_ell_delta_a+P.Jxz*P.C_n_delta_a)/bigGamma);

models.G_da2p = models.a_phi2/(s+models.a_phi1);
models.G_da2phi = models.G_da2p/s;

Vg = Va0;

models.G_phi2chi = P.gravity/s/Vg;

%
% Elevator channel coefficients and models
%

models.a_theta1 = -(rho*Va2*S*P.c/2) * (1/P.Jy) * (P.c/2/Va0) * P.C_m_q;
models.a_theta2 = -(rho*Va2*S*P.c/2) * (1/P.Jy) * P.C_m_alpha;
models.a_theta3 = (rho*Va2*S*P.c/2) * (1/P.Jy) * P.C_m_delta_e;

models.G_de2q = models.a_theta3*s / (s*s + models.a_theta1*s + models.a_theta2);
models.G_de2theta = models.G_de2q * (1/s);
models.G_theta2h  = Va0 / s;

%
% Throttle channel coefficients and models
%

models.a_V1 = -(rho *P.C_prop*P.S_prop/P.mass) * (P.delta_t0 * (1-2*P.delta_t0)*(P.k_motor - Va0) - P.delta_t0*Va0) + (rho*Va0*S/P.mass) * (P.C_D_0 + P.C_D_alpha*P.alpha0 + P.C_D_delta_e*P.delta_e0);
models.a_V2 = (rho*P.C_prop*P.S_prop/P.mass) * (P.k_motor-Va0) * (Va0 + 2*P.delta_t0*(P.k_motor - Va0));
models.a_V3 = P.gravity * cos(P.theta0 - P.alpha0);

models.G_dt2Va = models.a_V2 / (s+models.a_V1);
models.G_theta2Va = -models.a_V3 / (s + models.a_V1);

end