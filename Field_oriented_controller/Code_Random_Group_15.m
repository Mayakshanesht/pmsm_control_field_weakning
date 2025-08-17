%% Mini project for Advanced Electrical Drives
function mini_project_AED()

    close all
    
    % machine parameters
    Psi_f = 90e-3;              % [Vs], Field flux linkage 
    L_sd = 200e-6;              % [H], Inductance in d-axis 
    L_sq = 500e-6;              % [H], Inductance in q-axis  
    i_max = 500;                % [A], Maximum stator current (amplitude) 
    U_dc = 350;                 % [V], Dc_link voltage 
    u_smax = U_dc/sqrt(3);      % [V], maximum stator voltage
    p = 4;                      % Pole pairs
    
    i_sc = Psi_f / L_sd;       % [A] Short circuit current
    kappa = i_sc / i_max;       % Short circuit current normalized
    chi = (L_sq-L_sd)/(2*L_sd); % Saliency

    % store the sulotions in arrays for plotting the result later
    i_sd_values = [];
    i_sq_values = [];
    omega_s_values = [];
    T_e_values = [];


    %% TASKS
    %
    %% Task 1.1
    % b)
    % calculate referencs currents for reference torque in Nm and speed
    %in rpm
    [i_sd_ref, i_sq_ref] = calc_reference_currents(70,6584)


    %% Task 1.2
    % a)
    % plot toque over speed
    % n = 0 ... 50000 -> omega = 0 ... ~21000
    draw_contours;
    plot_current_trajectory(70,21000);

    % b)
    plot_torque_over_speed_map(70,21000);

    % c)
    max_n_A = round((get_max_omega_A(70)*60)/(p*2*pi),0);
    fprintf('Maximum speed for base speed operation with reference torque T_e* = %d Nm is %d rpm\n', 70, max_n_A)

    % d)
    max_n_B = round((get_max_omega_B(70)*60)/(p*2*pi),0);
    fprintf('Maximum speed for reference torque T_e* = %d Nm is %d rpm\n', 70, max_n_B)


    %% Task 1.3 b)
%     close all
    % n = 0 ... 50000 -> omega = 0 ... ~21000
    % calculate and plot task before
    draw_contours;
    plot_current_trajectory(70,21000);
    plot_torque_over_speed_map(70,21000);


    % adjust specs, reset arrays for solutions
    i_max = 400;
    kappa = i_sc / i_max;
    i_sd_values = [];
    i_sq_values = [];
    omega_s_values = [];
    T_e_values = [];
    % calculate and plot for new i_max
    % n = 0 ... 50000 -> omega = 0 ... ~21000
    plot_current_trajectory(70,21000);
    plot_torque_over_speed_map(70,21000);



    %% Helper function to calculate reference current and plot it
    

    % function to generate i_sdq for a given reference torque T_e and omega
    % creates array with all values for i_sd/q from start untill omega_ref is reached
    % -> use arrays with solution for plotting
    function calc_current_trajectory(T_e_ref, omega_s_ref)

        % reset arrays for solutions
        i_sd_values = [];
        i_sq_values = [];
        omega_s_values = [];
        T_e_values = [];

        % check if reference torque is feasible
        T_e_max = calc_maximum_torque();
        if T_e_ref > T_e_max
            disp('reference torque can´t be reached because of i_max limitation')
            fprintf('Max torque T_e = %i\n', T_e_max)
            return
        end
        

        % calculate i_sdq in base speed operation
        % increase torque untill T_e_ref has been reached
        % initialize T_e
        T_e = 0;
        while T_e <= T_e_ref

            [i_sd_ref, i_sq_ref] = calc_i_ref_MTPA(T_e);

            i_sd_values(end+1) = [i_sd_ref];
            i_sq_values(end+1) = [i_sq_ref];
            
            T_e = T_e+1;

        end
        
        % calculate max omegas
        omega_A = get_max_omega_A(T_e_ref);
        omega_B = get_max_omega_B(T_e_ref);

        % set omega_s so calculation start precisely at omega_A and not where previous iteration ended
        omega_s = omega_A;

        % basic field weakening
        % calculate i_sdq on the constant line of torque, until omega_B has been reached
        while omega_s <= omega_B && omega_s <= omega_s_ref
            
            [i_sd_ref, i_sq_ref] = calc_i_s_ref_LCT(T_e_ref, omega_s);
            i_sd_values(end+1) = [i_sd_ref];
            i_sq_values(end+1) = [i_sq_ref];

            T_e_values(end+1) = (3/2)*(i_sq_ref*Psi_f+(L_sd-L_sq)*i_sq_ref*i_sd_ref);

            omega_s_values(end+1) = omega_s;

            omega_s = omega_s + 100;

        end

        % field weakening
        % set omega_s so calculation start precisely at omega_B and not where previous iteration ended
        omega_s = omega_B;

        % if kappa >= 1 current trajectory runs entirely on MA circle after
        % omega_B and doesn´t follow MPTF after omega_C has been reached
        if kappa >= 1

            % operation for kappa >= 1 only possible until MF Ellipse crosses MA circle
            omega_MA_max = get_omega_max;

            while omega_s <= omega_s_ref && omega_s <= omega_MA_max
            
                [i_sd_ref, i_sq_ref] = calc_i_s_ref_MA(omega_s);
                i_sd_values(end+1) = [i_sd_ref];
                i_sq_values(end+1) = [i_sq_ref];
    
                T_e_values(end+1) = (3/2)*(i_sq_ref*Psi_f+(L_sd-L_sq)*i_sq_ref*i_sd_ref);
    
                omega_s_values(end+1) = omega_s;
    
                omega_s = omega_s + 500;    

            end

        else            % if kappa < 1

            % current trajectory follows on MA circle until omega_C then MTPF line
            omega_C = get_max_omega_C;
            while omega_s <= omega_C && omega_s <= omega_s_ref
                
                [i_sd_ref, i_sq_ref] = calc_i_s_ref_MA(omega_s);
                i_sd_values(end+1) = [i_sd_ref];
                i_sq_values(end+1) = [i_sq_ref]; 
    
                T_e_values(end+1) = (3/2)*(i_sq_ref*Psi_f+(L_sd-L_sq)*i_sq_ref*i_sd_ref);
    
                omega_s_values(end+1) = omega_s;
    
                omega_s = omega_s + 200;
    
            end
            
            % set omega_s
            % set omega_s so calculation start precisely at omega_C and not where previous iteration ended
            omega_s = omega_C;
            
            while omega_s <= omega_s_ref

                [i_sd_ref, i_sq_ref] = calc_i_s_ref_MPTF(omega_s);
                i_sd_values(end+1) = [i_sd_ref];
                i_sq_values(end+1) = [i_sq_ref];
    
                T_e_values(end+1) = (3/2)*(i_sq_ref*Psi_f+(L_sd-L_sq)*i_sq_ref*i_sd_ref);
    
                omega_s_values(end+1) = omega_s;
    
                omega_s = omega_s + 500;

            end

        end
   
    end

    % calculate i_sdq_ref for a given reference toque and speed
    % only one values each for i_sd/q, no array with solutions like in calc_current_trajectory(T_e_ref, omega_s_ref)
    function [i_sd_ref, i_sq_ref] = calc_reference_currents(T_e_ref, omega_s_ref)

        % check if referencs torque is feasible
        T_e_max = calc_maximum_torque();
        if T_e_ref > T_e_max
            disp('reference torque can´t be reached because of i_max limitation')
            fprintf('Max torque T_e = %i', T_e_max)
            i_sd_ref = NaN;
            i_sq_ref = NaN;
            return
        end

        % calculate max omegas
        omega_A = get_max_omega_A(T_e_ref);
        omega_B = get_max_omega_B(T_e_ref);

        % if kappa >= 1 current trajectory runs entirely on MA circle after
        % omega_B and doesnt follow MPTF after omega_C has been reached
        if kappa >= 1

            % operation for kappa >= 1 only possible until MF Ellipse crosses MA circle
            omega_MA_max = get_omega_max;

            if omega_s_ref >= omega_B || omega_s_ref <= omega_MA_max
                [i_sd_ref, i_sq_ref] = calc_i_s_ref_MA(omega_s_ref);
            elseif omega_s_ref >= omega_A
                [i_sd_ref, i_sq_ref] = calc_i_s_ref_LCT(T_e_ref, omega_s_ref);
            else
                [i_sd_ref, i_sq_ref] = calc_i_ref_MTPA(T_e);
            end

        end

        % calculation for kappa < 1
        if kappa < 1

            omega_C = get_max_omega_C();

            if omega_s_ref >= omega_C
                [i_sd_ref, i_sq_ref] = calc_i_s_ref_MPTF(omega_s_ref);
            elseif omega_s_ref >= omega_B
                [i_sd_ref, i_sq_ref] = calc_i_s_ref_MA(omega_s_ref);
            elseif omega_s_ref >= omega_A
                [i_sd_ref, i_sq_ref] = calc_i_s_ref_LCT(T_e_ref, omega_s_ref);
            else
                [i_sd_ref, i_sq_ref] = calc_i_ref_MTPA(T_e_ref);
            end

        end

    end

%% Task 1.2 b


    % function to generate and plot i_sdq for a given reference torque T_e and omega
    function plot_current_trajectory(T_e_ref, omega_s_ref)
    
        % call function to calculate i_sdq trajectory
        calc_current_trajectory(T_e_ref, omega_s_ref);

        % plot the results
        figure(1);
        plot(i_sd_values, i_sq_values, 'k', 'DisplayName', 'i\_sdq' , LineWidth=2);
        text(-i_max+30,-150+i_max/2,sprintf('imax = %d A', i_max));

    end

    % function to plot torque over speed
    function plot_torque_over_speed_map(T_e_ref, omega_s_ref)
        
        % create figure
        figure(2);
        xlabel 'Speed in rpm'
        ylabel 'Torque in Nm'
        title('torque trajectory')
        grid on
        axis([0,25000, 0, T_e_ref*p+15])
        set(gcf, 'Position',[300 400 800 500])
        hold on

        % calculate i_sdq trajectory for a given reference tourqe and max speed
        calc_current_trajectory(T_e_ref, omega_s_ref);

        % plot reference torque line
        plot([0, 50000],[T_e_ref, T_e_ref], 'b--', LineWidth=1);


        % plot T_e
        % plot ramp from 0->A
        % convert omega to n
        omega_A = get_max_omega_A(T_e_ref);
        n_A = round(omega_A*60)/(p*2*pi);
        plot([0, n_A],[0, T_e_ref], 'b', LineWidth=2);
        % plot torque over omega A->D
        plot(omega_s_values.*60/(p*2*pi),T_e_values, 'b', LineWidth=2);
        text(7500,1+i_max*0.1,sprintf('imax = %d A', i_max));


        % plot T_m
        % plot ramp from 0->A
        plot([0, n_A],[0, T_e_ref*p], 'r', LineWidth=1);
        % plot torque over omega A->D
        plot(omega_s_values.*60/(p*2*pi),T_e_values*p, 'r', LineWidth=1);
        text(5000,50+i_max*0.4,sprintf('imax = %d A', i_max));

        % create nice legend, since lines are fractured
        LH(1) = plot(nan, nan, 'b');
        L{1} = 'torque T\_e';
        LH(2) = plot(nan, nan, 'b--');
        L{2} = 'reference torque T\_e*';
        LH(3) = plot(nan, nan, 'r');
        L{3} = 'T\_m';

        legend(LH, L, 'Location','bestoutside')
        legend('-DynamicLegend');

    end

    % function to calculate the maximum possible torque
    % set up equation solver and solve for i_sdq -> calculate T_e_max
    % max T_e at crossing of MA circle and MPTA line
    function T_e_max = calc_maximum_torque()

        syms i_sd i_sq
        eq1 = sqrt(i_sd^2+i_sq^2) == i_max;
        eq2 = i_sd == (kappa/(8*chi)-sqrt(((sqrt(i_sq^2+i_sd^2)/i_max)^2)/2+(kappa/(8*chi))^2))*i_max;
        eq3 = i_sq >= 0;
        eqns = [eq1 eq2 eq3];
        [i_sd, i_sq] = solve(eqns, [i_sd i_sq], 'Real',true);

        T_e_max = 3/2*(i_sq*Psi_f) + 3/2*(L_sd-L_sq)*i_sq*i_sd;
        T_e_max = vpa(T_e_max,5);

    end

    % function to calculate reference current for MPTA operation
    % 0 -> A, base speed, MTPA
    % per exrcise 5.1
    function [i_sd_ref, i_sq_ref] = calc_i_ref_MTPA(T_e)
         
        % polynom with parameters
        a = (L_sd-L_sq)^2;
        b = 0;
        c = 0;
        d = 2/3 * T_e * Psi_f;
        e = -(2/3* T_e)^2;
        
        % only positive real values
        A = roots([a, b, c, d, e]);
        i_sq_ref = min(A(real(A)>=0&imag(A)==0));
        i_sd_ref = (2/3*T_e-Psi_f*i_sq_ref) / (i_sq_ref*(L_sd-L_sq));

    end

    % function to calculate i_s_ref along the Lines of Constant Torque
    % set up equation solver and solve for i_sdq for given T_e and omega_s
    function [i_sd_ref, i_sq_ref] = calc_i_s_ref_LCT(T_e, omega_s)
   
        syms i_sd_ref 
        syms i_sq_ref positive
        eq1 = (i_sd_ref+i_sc)^2+i_sq_ref^2*(2*chi+1)^2 == (u_smax/(omega_s*L_sd))^2;
        eq2 = T_e == 3/2*(i_sq_ref*Psi_f) + 3/2*(L_sd-L_sq)*i_sq_ref*i_sd_ref;
        eq3 = sqrt(i_sd_ref^2+i_sq_ref^2) <= i_max;
        eqns = [eq1 eq2 eq3];
        [i_sd_ref, i_sq_ref] = solve(eqns, [i_sd_ref i_sq_ref], 'Real',true);

        i_sd_ref = vpa(i_sd_ref,5);
        i_sq_ref = vpa(i_sq_ref,5);

    end

    % function to calculate i_s_ref on the MA circle and MF Ellipse
    % set up equation solver and solve for i_sdq for given omega_s
    function [i_sd_ref, i_sq_ref] = calc_i_s_ref_MA(omega_s)
   
        syms i_sd_ref
        syms i_sq_ref positive
        eq1 = (i_sd_ref+i_sc)^2+i_sq_ref^2*(2*chi+1)^2 == (u_smax/(omega_s*L_sd))^2;
        eq2 = sqrt(i_sd_ref^2+i_sq_ref^2) == i_max;
        eqns = [eq1 eq2];
        [i_sd_ref, i_sq_ref] = vpasolve(eqns, [i_sd_ref i_sq_ref]);

    end

    % function to calculate i_s_ref along the MTPF line and MF Ellipse
    % set up equation solver and solve for i_sdq for given omega_s
    function [i_sd_ref, i_sq_ref] = calc_i_s_ref_MPTF(omega_s)
   
        syms i_sd_ref
        syms i_sq_ref positive
        eq1 = (i_sd_ref+i_sc)^2+i_sq_ref^2*(2*chi+1)^2 == (u_smax/(omega_s*L_sd))^2;
        eq2 = i_sd_ref == (-kappa + (1+2*chi)*kappa/(8*chi)-sqrt((((u_smax/omega_s)/(L_sd*i_max))^2)/2+((1+2*chi)*kappa/(8*chi))^2))*i_max;
%         eq3 = i_sq_ref == ((1/(2*chi+1))*sqrt(((u_smax/omega_s)/(L_sd*i_max))^2-(kappa+i_sd_ref/i_max)^2))*i_max;
        eqns = [eq1 eq2];
        [i_sd_ref, i_sq_ref] = solve(eqns, [i_sd_ref i_sq_ref], 'Real', true);
 
        i_sd_ref = vpa(i_sd_ref,5);
        i_sq_ref = vpa(i_sq_ref,5);

    end

    % function to determinate the maximum omega_A
    % calculate i_sdq in crossing of lines of MTPA line and line constant torque then calculate omega_A
    % set up equation solver and solve for omega_A for given T_e
    function omega_A = get_max_omega_A(T_e)
        
        syms i_sd_ref i_sq_ref 
        eq1 = T_e == (3/2)*(i_sq_ref*Psi_f+(L_sd-L_sq)*i_sq_ref*i_sd_ref);
        eq2 = i_sd_ref == (kappa/(8*chi)-sqrt(((sqrt(i_sq_ref^2+i_sd_ref^2)/i_max)^2)/2+(kappa/(8*chi))^2))*i_max;
        eqns = [eq1 eq2];
        [i_sd_ref, i_sq_ref ] = solve(eqns, [i_sd_ref i_sq_ref], 'Real',true);
        
        i_sd_ref = min(i_sd_ref);
        i_sq_ref = min(i_sq_ref);
        
        Psi_s = sqrt((Psi_f + L_sd*i_sd_ref)^2 + (L_sq*i_sq_ref)^2);
        omega_A = vpa(u_smax / Psi_s,6);

    end

    % function to determinate the maximum omega_B
    % calculate i_sdq in crossing of lines of constant torque and MA Circle then calculate omega_B
    % set up equation solver and solve for omega_B for given T_e 
    function omega_B = get_max_omega_B(T_e)
        
        syms i_sd_ref i_sq_ref
        eq1 = T_e == (3/2)*(i_sq_ref*Psi_f+(L_sd-L_sq)*i_sq_ref*i_sd_ref);
        eq2 = sqrt(i_sd_ref^2+i_sq_ref^2) == i_max;
        eqns = [eq1 eq2];
        [i_sd_ref, i_sq_ref] = solve(eqns, [i_sd_ref i_sq_ref], 'Real',true);
        
        i_sd_ref = min(i_sd_ref);
        i_sq_ref = min(i_sq_ref);
        
        Psi_s = sqrt((Psi_f + L_sd*i_sd_ref)^2 + (L_sq*i_sq_ref)^2);
        omega_B = vpa(u_smax / Psi_s,5);

    end

    % function to determinate the maximum omega_C
    % calculate i_sdq in crossing of lines of MA Circle and MTPF then solve for omega_C
    % set up equation solver and solve for omega_C
    function omega_C = get_max_omega_C()

        syms i_sd_ref i_sq_ref omega_C
        eq1 = i_sd_ref == (-kappa + (1+2*chi)*kappa/(8*chi)-sqrt((((u_smax/omega_C)/(L_sd*i_max))^2)/2+((1+2*chi)*kappa/(8*chi))^2))*i_max;
        eq2 = i_sq_ref == ((1/(2*chi+1))*sqrt(((u_smax/omega_C)/(L_sd*i_max))^2-(kappa+i_sd_ref/i_max)^2))*i_max;
        eq3 = sqrt(i_sd_ref^2+i_sq_ref^2) == i_max;
        eq4 = i_sd_ref <= 0;
        eq5 = i_sq_ref >= 0;
        eq6 = omega_C >= 0;
        eqns = [eq1 eq2 eq3 eq4 eq5 eq6];
        [i_sd_ref, i_sq_ref, omega_C] = solve(eqns, [i_sd_ref i_sq_ref omega_C], 'Real',true);

        omega_C = vpa(omega_C,9);

    end

    % function to determinate the maximum omega for kappa >= 1
    % when MF Ellipse shrinks and only touches MA circle
    % set up equation solver and solve for omega_max on the crossing of MA circle and MF Ellipse
    % ONLY FOR KAPPA > 1
    function omega_MA_max = get_omega_max()

        syms i_sd_ref
        syms omega_max
        syms i_sq_ref positive
        eq1 = (i_sd_ref+i_sc)^2+i_sq_ref^2*(2*chi+1)^2 == (u_smax/(omega_max*L_sd))^2;
        eq2 = sqrt(i_sd_ref^2+i_sq_ref^2) == i_max;
        eq3 = i_sd_ref == -i_max;
        eqns = [eq1 eq2 eq3];
        [i_sd_ref, i_sq_ref, omega_max] = vpasolve(eqns, [i_sd_ref i_sq_ref, omega_max]);

        omega_MA_max = omega_max;

    end
   
   %% Plots
   
    % function to draw all relevant contours
    % MA Circle, Lines of constant torque, MTPA line, MF ellipse, MPTF line
    % and mark the short cicuit current -i_sq
    function draw_contours()
        
        % create figure
        figure(1);
        xlabel 'i\_sd in A'
        ylabel 'i\_sq in A'
        title('current trajectory')
        grid on
        axis([-800,0,0,600])
        set(gcf,'Position',[400 400 800 500])
        hold on

        % set up for legend, default is kinda scuffed
        LH(1) = plot(nan, nan, 'r');
        L{1} = 'MA';
        LH(2) = plot(nan, nan, 'k--');
        L{2} = 'const. torque';
        LH(3) = plot(nan, nan, 'r--');
        L{3} = 'MTPA';
        LH(4) = plot(nan, nan, 'g:');
        L{4} = 'MF Ellipse';
        LH(5) = plot(nan, nan, 'b+');
        L{5} = 'i\_sc';
        LH(6) = plot(nan, nan, 'g--');
        L{6} = 'MTPF';

        % plot the functions and equity lines
        draw_MA_circle([400 500])
        draw_const_torque()
        draw_MTPA_line()
        draw_MF_ellipse()
        draw_short_circuit_current()
        draw_MTPF_line()

        legend(LH, L, 'Location','bestoutside')
 
    end

    % function to draw the MA circle for given i_max
    function draw_MA_circle(i_max)
    
        % define function
        zfun = @(i_sd,i_sq) sqrt(i_sd.^2 + i_sq.^2);
        zhandle = fcontour(zfun, 'r');
        
        % creates contour on equity line
        zhandle.LevelList = i_max;

    end
    
    % function to draw the theoritical MTPA line
    function draw_MTPA_line()
    
        i_sd_MPTA = [];
        i_sq_MTPA = [];

        % evaluate 
        for T_e=1:1:180


            [i_sd_ref,i_sq_ref] = calc_i_ref_MTPA(T_e);
            i_sd_MPTA(end+1) = i_sd_ref;
            i_sq_MTPA(end+1) = i_sq_ref;
            
        end

            plot(i_sd_MPTA,i_sq_MTPA,'r--')

    end

    % function to draw the constant torque line
    function draw_const_torque()
    
        zfun = @(i_sd,i_sq) (3/2)*(i_sq*Psi_f+(L_sd-L_sq)*i_sd.*i_sq);
        %zfun = @(i_sd,i_sq) (3/2)*Psi_f*i_sq*(1-(2*chi/i_sc)*i_sd);
        zhandle = fcontour(zfun, 'k--');

        % creates contour on equity line
        zhandle.LevelList = [5, 30, 70, 108];

    end

    % function to draw the MF_ellipse
    function draw_MF_ellipse()
    
        % define function for MF Ellipse
        zfun = @(i_sd,i_sq) (i_sd+i_sc).^2+i_sq.^2*(2*chi+1).^2;
        zhandle = fcontour(zfun, 'g:', 'LineWidth', 1.5);
        
        % creates contour on equity line
        % for T_e* = 70 Nm relevant lines:
        %   omega_A = 1229
        %   omega_B = 1963
        %   omega_C = 5240
        %   omega_m = 20207
        zhandle.LevelList = [(u_smax/(1229*L_sd))^2, (u_smax/(1963*L_sd))^2, (u_smax/(5240*L_sd))^2, (u_smax/(20207*L_sd))^2];

    end

    % function to draw the MTPF line
    function draw_MTPF_line()
        
        % create arrays to save solutions along the MPTF line for plotting
        i_sd_MPTF = [];
        i_sq_MPTF = [];

        for n = 0:100:50000
            
            omega_s = (n*p*2*pi)/60;         % convert n to omega

            Psi_s_max = u_smax/omega_s;
            i_O = Psi_s_max/(L_sd*i_max);
            
            i_sdn = -kappa + (1+2*chi)*kappa/(8*chi)-sqrt((i_O^2)/2+((1+2*chi)*kappa/(8*chi))^2);
            i_sd_MPTF(end+1) = i_sdn * i_max;
            i_sqn = (1/(2*chi+1))*sqrt(i_O^2-(kappa+i_sdn)^2);
            i_sq_MPTF(end+1) = i_sqn * i_max;
            
        end

        plot(i_sd_MPTF,i_sq_MPTF,'g--', 'LineWidth', 1.5);

    end

    % function to mark -i_sc on graph
    function draw_short_circuit_current()
        
        plot(-i_sc, 0, 'b+', LineWidth=2);
        text(-i_sc, 0, '-i\_sc','VerticalAlignment','bottom','HorizontalAlignment','left');

    end
end



