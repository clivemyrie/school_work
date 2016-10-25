function lect7_1
%%
clc; clear; close all

load_uavsim

P.Va0 = 13;

models = compute_tf_models(P);

G_da2p = models.G_da2p
G_da2phi = models.G_da2phi
G_de2q = models.G_de2q
G_de2theta = models.G_de2theta
G_dt2Va = models.G_dt2Va
G_phi2chi = models.G_phi2chi
G_theta2h = models.G_theta2h
G_theta2Va = models.G_theta2Va


end