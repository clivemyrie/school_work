function lect5_3
%%
clc;clear all
load_uavsim

max_va = 0.0;
min_va = 10000000.0;

for va = 6:0.1:18
    
    P.Va0 = va;
    [~, trim_solution] = compute_longitudinal_trim(P);
    if trim_solution.valid
        disp('======================================')
        disp(['trim_solution.valid = ' num2str(trim_solution.valid)] )
        disp(['va = ' num2str(va)] )
        alpha = trim_solution.alpha;
        delta_e = trim_solution.delta_e;
        delta_t = trim_solution.delta_t;
        disp('======================================')
        if(abs(alpha)<=30 && abs(delta_e)<=45 && (delta_t>=0 && delta_t<=1)) % viable solution
            min_va = min(min_va, va);
            max_va = max(max_va, va);
        end
    end
end

clc;

disp(['min air speed ' num2str(min_va) ' m/s'])
P.Va0 = min_va;
compute_longitudinal_trim(P);

disp(['max air speed ' num2str(max_va) ' m/s'])
P.Va0 = max_va;
compute_longitudinal_trim(P);
end