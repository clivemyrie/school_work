function y = lect4_2f(gamma)

load_uavsim
star = 0.5 * P.rho * P.Va0 * P.Va0 * P.S_wing;
y = P.C_m_0 + (P.C_m_alpha/P.C_D_alpha)*(-P.mass*P.gravity*sin(gamma)/star-P.C_D_0) + (P.C_m_delta_e/P.C_L_delta_e)*(P.mass*P.gravity*cos(gamma)/star - P.C_L_0);

end