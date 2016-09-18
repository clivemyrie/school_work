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

    %
    % Aileron channel coefficients and models
    %

    models.a_phi1 = nan;
    models.a_phi2 = nan;

    models.G_da2p = models.a_phi2/(s+models.a_phi1);
    models.G_da2phi = models.G_da2p/s;

    models.G_phi2chi = nan;
        
    %
    % Elevator channel coefficients and models
    %

    models.a_theta1 = nan;
    models.a_theta2 = nan;
    models.a_theta3 = nan;

    models.G_de2q = nan;
    models.G_de2theta = nan;
    models.G_theta2h  = nan;

    %
    % Throttle channel coefficients and models
    %

    models.a_V1 = nan;
    models.a_V2 = nan;
    models.a_V3 = nan;

    models.G_dt2Va = nan;
    models.G_theta2Va = nan;
    
end