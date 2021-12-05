%{
AAE 421: Fall 2021
Final Project - Blimp Controller

Contributers:
	Brendan Gillis	Jeremy Casella	Brandon Dimitri
	Martino Gogna	Jared DePerna	Jonathan Friedrich
	Kaustubh Ray	Matt Kim		Saketh Nibhanupudi

Description:
	Setups and runs the 'blimp_simulation.slx' simulink file
	with a variety of configuration settings

TODO:
	- Plot root locus with proportional gain Kp > 0 
	- Automate pulling stats for SS_error, overshoot and settling time
	- Don't know how the 0-255 control input relates to a target position
	- Compute cumulative distance error for circle tracking
%}




%Sample input below: Opp modes are: 
%						'altitude hold', 'heading hold' 'circle track'

% simulate_blimp(part_num, closed_loop, controller_on, t_final,		   opp_mode)
%simulate_blimp(		 'di',		  true,			 true,     100, 'altitude hold');
simulate_blimp(		'dii',		  true,			 true,     100,  'heading hold');
%simulate_blimp(	   'diii',		  true,			 true,     180,  'circle track');


function simulate_blimp(part_num, closed_loop, controller_on, t_final, opp_mode)
	
	% This just lets us call the simulink file from this function
	options = simset('SrcWorkspace','current');

	% Gains found user PID Tuner
	[Kp_a, Ki_a, Kd_a, N_a] = deal(38, 16, 74, 1.5);
	[Kp_h, Ki_h, Kd_h, N_h] = deal(75, 0.05, -11, 3.44);
	[Kp_f, Ki_f, Kd_f, N_f] = deal(96, 17, -67.5, 1.1);

	% freq to switch between velo and heading control [hz]
	switch_freq		= 5;

    % Speed Catch Up Multiplier for Circle Tracking, 0 when not circle
    speed_mult      = 0;

	% Changes weight of heading error
	switch_weight	= 20;

	%% DEFINE THE TRANSFER FUNCTIONS
	s	= tf('s');

	% Altitude [m] / Actuator PWM input - (ref = 1 m)
	Ga	= (2.423*s+0.1097) / (s^3+0.357*s^2+0.1394*s+0.002024) * 1/1000;
	
	% Heading [rad] / Actuator PWM input (-255 to 255) - (ref = 0 rad)
	Gh	= (0.00835*s+0.001745) / (s^2+4.641E-5*s+0.0002839);
	
	% Forward speed [m/s] / Actuator PWM input (0 to 255) - (ref= 0.3 m/s)
	Gf	= (0.0002557*s^2 - 0.02031*s + 0.05613) / (s^3 + 0.4456*s^2 + 27.55*s + 4.316);
	
	%% SET UP INPUT VALUES AND GOALS
	n		= 2^12; % size of target signal arrays (used in graphs)
	t_goal	= linspace(0, t_final, n); % Used to make graphs of target signal
	radius	= 8; % Radius of circle [m]

	% Default values in case not used
	target_head		= 0; 
	slope			= 0;

	if strcmp(opp_mode, 'altitude hold')
		opp_mode		= 1;
		target_alt		= 1;	% [m]
		target_head		= 0;	% [rad]
		target_speed	= 0;	% [m/s]	
		ya_goal			= target_alt * ones(n, 1);
		yh_goal			= target_head * ones(n, 1);
		yf_goal			= target_speed * ones(n, 1);
	
	elseif strcmp(opp_mode,'heading hold')
		opp_mode		= 2;
		target_alt		= 1;	% [m]
		target_head		= pi/2;	% [rad]
		target_speed	= 1.3;	% [m/s]
		ya_goal			= target_alt * ones(n, 1);
		yh_goal			= target_head * ones(n, 1);
		yf_goal			= target_speed * ones(n, 1);
	
	elseif strcmp(opp_mode, 'circle track')
		opp_mode		= 3;   
		target_alt		= 0;	% [m]
		target_speed	= 0.3;	% [m/s]
        speed_mult      = 0.05;
		slope			= target_speed/radius; % [rad/s]
		ya_goal			= target_alt * ones(n, 1);
		yh_goal			= t_goal * slope;
		yf_goal			= target_speed * ones(n, 1);
	
	end
	
	%% RUN THE SIMULATION
	results = sim("blimp_simulation.slx", t_final, options);
	t_sim	= results.tout;
	ya		= results.ya.signals.values;
	yh		= results.yh.signals.values;
	yf		= results.yf.signals.values;
	xPos	= results.xPos.signals.values;
	yPos	= results.yPos.signals.values;

	% If we simulated more than 1 circle grab the first circle
	xPos	= xPos(yh<2*pi);
	yPos	= yPos(yh<2*pi);
	t_sim	= t_sim(yh<2*pi);
	t_sim	= t_sim - t_sim(1);

	%% PLOT AND SAVE SIMULAION OUTPUT
	if not(isfolder('figures'))
		mkdir figures
	end
	
	if opp_mode == 1
		fig = figure();
		heading		= strcat("Part ", part_num, ": Altitude (m)");
		savename	= strcat("Part", part_num, "_Altitude");
		hold on
		set(gca, 'Box', 'on');
		title(heading, 'Fontsize', 12);
		xlabel('Time (s)', 'Fontsize', 10);
		ylabel('Altitude (m)', 'Fontsize', 10);
		plot(t_goal, ya_goal, t_sim, ya,  'linewidth', 1);
		legend('Target Values', 'Simulated Values');
		save_figure(fig, savename, 'figures\');
	end
	
	if opp_mode == 2 
		fig = figure();
		heading		= strcat("Part ", part_num, ": Heading");
		savename	= strcat("Part", part_num, "_Heading");
		hold on
		set(gca, 'Box', 'on');
		title(heading, 'Fontsize', 12);
		xlabel('Time (s)', 'Fontsize', 10);
		ylabel('Heading (rad)', 'Fontsize', 10);
		plot(t_goal, yh_goal, t_sim, yh,  'linewidth', 1);
		legend('Target Values', 'Simulated Values');
		save_figure(fig, savename, 'figures\');
	end
	
	if opp_mode == 3
		fig = figure();
		heading		= strcat("Part ", part_num, ": Circle Tracking");
		savename	= strcat("Part", part_num, "_Circl");
		hold on
		set(gca, 'Box', 'on');
		title(heading, 'Fontsize', 12);
		xlabel('X Position (m)', 'Fontsize', 10);
		ylabel('Y Positin (m)', 'Fontsize', 10);
		a	= viscircles([0, radius], radius);
		b	= plot(xPos, yPos,  'linewidth', 1);
		legend([a, b], {'Target Values', 'Simulated Values'});
		save_figure(fig, savename, 'figures\');

		fig = figure();
		heading		= strcat("Part ", part_num, ": Circle Tracking - X");
		savename	= strcat("Part", part_num, "_Circl_x");
		hold on
		set(gca, 'Box', 'on');
		title(heading, 'Fontsize', 12);
		xlabel('Time (s)', 'Fontsize', 10);
		ylabel('X Position (m)', 'Fontsize', 10);
		a	= plot(t_sim, radius*sin(slope*t_sim),  'linewidth', 1);
		b	= plot(t_sim, xPos,  'linewidth', 1);
		legend([a, b], {'Target Values', 'Simulated Values'});
		save_figure(fig, savename, 'figures\');

		fig = figure();
		heading		= strcat("Part ", part_num, ": Circle Tracking - Y");
		savename	= strcat("Part", part_num, "_Circl_y");
		hold on
		set(gca, 'Box', 'on');
		title(heading, 'Fontsize', 12);
		xlabel('Time (s)', 'Fontsize', 10);
		ylabel('Y Positin (m)', 'Fontsize', 10);
		a	= plot(t_sim, radius*cos(slope*t_sim+pi)+radius,  'linewidth', 1);
		b	= plot(t_sim, yPos,  'linewidth', 1);
		legend([a, b], {'Target Values', 'Simulated Values'});
		save_figure(fig, savename, 'figures\');

		y = radius*cos(slope*t_sim+pi)+radius;
		x = radius*sin(slope*t_sim);

		% 
		xPos = xPos(t_sim > 40)
		yPos = yPos(t_sim > 40)
		x = x(t_sim > 40)
		y = y(t_sim > 40)
		t_sim = t_sim(t_sim > 40) - 40;

		buffer = [0; t_sim];
		error = sum(sqrt((xPos-x).^2 + (yPos-y).^2) .* (t_sim-buffer(1:end-1)))
		avg_error = error / t_sim(end)


	end

end


function save_figure(fig, name, location)
	% this just helps us save neat and consistant figures
	% with minimal whitespace
	name		= [char(name) '.png'];
    ax			= get(fig,'CurrentAxes');
    Position	= get(ax,'Position');
    outerpos	= get(ax,'OuterPosition');
    ti			= get(ax,'TightInset'); 
    left		= outerpos(1) + ti(1);
    bottom		= outerpos(2) + ti(2);
    ax_width	= outerpos(3)*0.999 - ti(1) - ti(3);
    ax_height	= outerpos(4)*0.999 - ti(2) - ti(4);
    set(ax,'Position',[left bottom ax_width ax_height]);

    print(fig,fullfile(location,name),'-dpng','-r300');       
    set(ax, 'Position', Position);
end



