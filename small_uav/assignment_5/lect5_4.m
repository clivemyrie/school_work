function lect5_4
%%
clc;clear all
load_uavsim;P.k_Tp = 5e-6;

trimmed_da = 0.0;
curr_min_cost = 100000000.0;
for da = linspace(-P.delta_a_max, P.delta_a_max, 100)
    P.delta_a0 = da;
    
    [~, trim_solution] = compute_longitudinal_trim(P);
    if trim_solution.valid && (trim_solution.cost < curr_min_cost)
        disp('======================================')
        disp(['trim_solution.valid = ' num2str(trim_solution.valid)] )
        disp(['da = ' num2str(da)] )
        disp('======================================')
        trimmed_da = da;
        curr_min_cost = trim_solution.cost;
    end
end

P.delta_a0 = trimmed_da;
trimmed_da
end
