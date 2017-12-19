classdef RunParameters

	properties

		rover_scale = 0.3;

		time_scale = 0.3;

		rovers_num = 4;

		timeout = Inf;

		delta_lambda = 0.2;

		delta_phi = 0.2;

		delta_alpha = 0.8;

		replan_prob = 0.25;

%		rover_positions = [32 86 98 64];

%		rover_destinations = [145 13 17 22];

		rover_positions = [14 47 70 16];

		rover_destinations = [152 51 46 10];

		stop_at_end = true;

		animate = true;

	end

	properties (Hidden = true)
		
		stop_level = 0;

		debugging_level = 0;

	end
end
