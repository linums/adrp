classdef Globals

	methods (Static)
		
		function ans=rover_scale(value)
			persistent Value;
			if nargin
				Value = value;
			end
			ans = Value;
		end

		function ans=path_size(value)
			persistent PathSize;
			if nargin 
				PathSize = value;
			end
			ans = PathSize;
		end

		function ans=time_scale(value)
			persistent Scale;
			if nargin 
				Scale = value;
			end
			ans = Scale;
		end

		function ans=path_size_ratio(value)
			persistent Ratio;
			if nargin
				Ratio = value;
			end
			ans = Ratio;
		end

		function ans=edge_probability(value)
			persistent EdgeProb;
			if nargin
				EdgeProb = value;
			end
			ans = EdgeProb;
		end

		function ans=rovers_num(value)
			persistent RoverNum;
			if nargin
				if value > 7 
					error('max num is 7');
				end
				RoverNum = value;
			end
			ans = RoverNum;
		end

		function ans=edge_weight(value)
			persistent EdgeWeight;
			if nargin
				EdgeWeight = value;
			end
			ans = EdgeWeight;
		end

		function ans=stop_at_round_end(value)
			persistent Stop;
			if nargin
				Stop = value;
			end
			ans = Stop;
		end

		function ans=debugging(value)
			persistent Debugging;
			if nargin
				Debugging = value;
			end
			ans = Debugging;
		end

	end
end
