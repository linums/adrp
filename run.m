function ans = run(map,rover_datas)

	load(map);

	%Displaying the map
	path_size = parameters.path_width*parameters.path_height;
	plot(G,'XData',vertices_pos(1:path_size,1),'YData',vertices_pos(1:path_size,2));
	axis equal;
	axis off;

	if nargin == 1
		%when no rover data was given
		run_params = RunParameters();
		rover_datas = add_rovers(map,run_params);
	else
	if nargin == 2
		if isa(rover_datas,'RunParameters')
		%when rover parameters are given
			rover_datas = add_rovers(map,rover_datas);
		end %if no case is matching than rover file is given
	end
	end

	ans = do(map,rover_datas);
end

function ans=add_rovers(map,run_params)
	load(map);
	path_size = parameters.path_height*parameters.path_width;
	
	%setting rover position vector
	reserved_vertices = false(path_size, 1);
	%setting array for possible rover colors
	colors = [ 'y' 'm' 'c' 'r' 'g' 'w' 'k' ];
	if run_params.rovers_num > 0 && run_params.rovers_num < path_size
		%creating rover display shape
		x = [ -1 -0.7 0 0.7 1 0.7 0 -0.7];
		y = [ 0 0.7 1 0.7 0 -0.7 -1 -0.7];
		%x = [ -1 -0.55 -0.1 0.35 0.8 0 0.8 0.35 -0.1 -0.55 ];
		%y = [ 0 0.835 0.995 0.93 0.6 0 -0.6 -0.93 -0.995 -0.835];

		for i = 1:size(x,2)
			x(i) = x(i) * run_params.rover_scale;
			y(i) = y(i) * run_params.rover_scale;
		end

		for i = 1:run_params.rovers_num;
			%rover structure
			%% *rover structure
			%%% *position - the rover position in the graph
			%%% *reserved - while moving, the destination what is reserved in the graph
			%%% *pg_position - the rover position in the planning graph - this is needed, because it includes the information about the rotation of the rover
			rover = struct( 'position' , NaN, 'reserved', NaN,  'pg_position', NaN,  'x', NaN, 'y', NaN , 'rotation' , NaN , 'handles' , struct( 'rover_patch', [], 'trajectory' , []), 'destination', NaN, 'tasks', [{Task()}], 'timeout_counter', 0);
			if isempty(run_params.rover_positions)
				temp = randi([ 1 path_size ]);
				if size(reserved_vertices(reserved_vertices == false),1) == 0
					fprintf('No avaiable node to put rover\n');
				end
				while reserved_vertices(temp) && size(reserved_vertices(reserved_vertices == false),1) ~= 0
					temp = randi([ 1 path_size ]);
				end

				rover.position = temp;
			else
				if reserved_vertices(run_params.rover_positions(i))
					error('wrong position datas');
				else
					rover.position = run_params.rover_positions(i);
				end
			end
			reserved_vertices(rover.position) = true;
			rover.x = vertices_pos(rover.position,1);
			rover.y = vertices_pos(rover.position,2);

			%rotating rovers to position in the planning graph
			rover.rotation = 0;
			%if rover.position+1 <= path_size && ceil(rover.position/parameters.path_height) == ceil((rover.position+1)/parameters.path_height) && adjacency_mtx(rover.position,rover.position+1) ~= 0
			%	rover.rotation = wrapToPi(atan2(vertices_pos(rover.position+1,2)-rover.y , vertices_pos(rover.position+1,1)-rover.x)-pi);
			%	rover.pg_position = PG.findnode(strcat(strcat(num2str(rover.position+1),'-'),num2str(rover.position)));
			%else
			%if rover.position+parameters.path_height <= path_size && adjacency_mtx(rover.position,rover.position+parameters.path_height) ~= 0
			%	rover.rotation = wrapToPi(atan2(vertices_pos(rover.position+parameters.path_height,2)-rover.y,vertices_pos(rover.position+parameters.path_height,1)-rover.x)-pi);
			%	rover.pg_position = PG.findnode(strcat(strcat(num2str(rover.position+parameters.path_height),'-'),num2str(rover.position)));
			%else
			%if rover.position-1 > 0 && ceil(rover.position/parameters.path_height) == ceil((rover.position-1)/parameters.path_height) &&  adjacency_mtx(rover.position,rover.position-1) ~= 0
			%	rover.rotation = wrapToPi(atan2(vertices_pos(rover.position-1,2)-rover.y,vertices_pos(rover.position-1,1)-rover.x)-pi);
			%	rover.pg_position = PG.findnode(strcat(strcat(num2str(rover.position-1),'-'),num2str(rover.position)));
			%else
			%if rover.position-parameters.path_height > 0 && adjacency_mtx(rover.position, rover.position-parameters.path_height) ~= 0
			%	rover.rotation = wrapToPi(atan2(vertices_pos(rover.position-parameters.path_height,2)-rover.y,vertices_pos(rover.position-parameters.path_height,1)-rover.x)-pi);
			%	rover.pg_position = PG.findnode(strcat(strcat(num2str(rover.position-parameters.path_height),'-'),num2str(rover.position)));
			%else
			%	rover.pg_position = NaN;
			%end 
			%end 
			%end
			%end
			rover.pg_position = rover.position;

			x_centered = rover.x+x;
			y_centered = rover.y+y;
			x_done = [1:size(x,2)];
			y_done = [1:size(y,2)];

			for j = 1:size(x,2)
				x_done(j) = x(j)*cos(rover.rotation)-y(j)*sin(rover.rotation);
				y_done(j) = x(j)*sin(rover.rotation)+y(j)*cos(rover.rotation);
			end
			x_centered = x_done + rover.x;
			y_centered = y_done + rover.y;
			%rover.handles.rover_patch = patch(x_centered,y_centered,colors(i));

			rovers(i) = rover;
			if run_params.debugging_level > 0 
				fprintf('[info]rover[%d]position:%d\n',i,rovers(i).position, 0);
			end
		end
		save 'rovers' 'rovers' 'x' 'y' 'run_params' 'colors' 'reserved_vertices'
		ans = 'rovers';
	end
end

function ans = init_destinations(rovers,G,map,rover_datas,run_params)
	global rover_handles;
	load(map);
	load(rover_datas);
	path_size = parameters.path_height*parameters.path_width;

	pg = convert(adjacency_mtx);
	pg_f = floyd(pg);

	for i = 1:size(rovers,2)
		if isempty(run_params.rover_destinations)
			rovers(i).destination = randi([1 path_size]);
			while rovers(i).destination == rovers(i).position || adjacency_mtx(rovers(i).destination,rovers(i).position) ~= 0
				rovers(i).destination = randi([1 path_size]);
			end
		else
			rovers(i).destination = run_params.rover_destinations(i);
		end
		%checking that existing path isselected
		if size(G.Edges.EndNodes(G.Edges.EndNodes == rovers(i).destination),1) ~= 0 
			clear temp;
			clear temp_num;
			temp_num = 0;
			%getting the shortest path to the selected vertex
			%for j = 1:size(G.Edges.EndNodes,1)
			%	if G.Edges.EndNodes(j,1) == rovers(i).destination
			%		temp(temp_num+1) = PG.findnode(strcat(strcat(num2str(G.Edges.EndNodes(j,2)),'-'),num2str(G.Edges.EndNodes(j,1))));
			%		temp_num = temp_num + 1;
			%	end
			%	if G.Edges.EndNodes(j,2) == rovers(i).destination
			%		temp(temp_num+1) = PG.findnode(strcat(strcat(num2str(G.Edges.EndNodes(j,1)),'-'),num2str(G.Edges.EndNodes(j,2))));
			%		temp_num = temp_num + 1;
			%	end
			%end
			
			%if temp_num ~= 0 && ~isinf(pg_f{1}(rovers(i).pg_position,temp(1)))
				%rovers(i).destination = temp(1);
				
				%for j = 2:temp_num
				%	if pg_f{1}(rovers(i).pg_position,temp(j)) < pg_f{1}(rovers(i).pg_position,rovers(i).destination)
				%		rovers(i).destination = temp(j);
				%	end
				%end

				rovers(i).tasks = get_tasks(pg_f, rovers(i).destination, rovers(i).pg_position, vertices_pos, PG);
				%displaying rover lines
				clear line_x;
				clear line_y;
				if ~isempty(rover_handles(i).trajectory)
					rover_handles(i).trajectory.delete;
				end
				line_x(1) = vertices_pos(rovers(i).position,1);
				line_y(1) = vertices_pos(rovers(i).position,2);
				%changed for version with no rotation
				for j = 2:2:size(rovers(i).tasks,2)
					%str = PG.Nodes.Name(rovers(i).tasks(j).goal);
					%res = strsplit(str{1},'-');
					destination = rovers(i).tasks(j).goal;
					%destination = str2num(res{2});
					line_x(j/2+1) = vertices_pos(destination,1);
					line_y(j/2+1) = vertices_pos(destination,2);
				end
				hold on;
				rover_handles(i).trajectory = plot(line_x,line_y,colors(i));
				hold off;

				if run_params.debugging_level > 0 
					fprintf('[info]destination:%d(pg)%d\n',rovers(i).destination, 0);
				end
			%else
			%	rovers(i).destination = NaN;
			%end
		else
			rovers(i).destination = NaN;
		end
		if isnan(rovers(i).destination)
			rovers(i).tasks = Task();
		end
	end

	for i = 1:size(rovers,2)
		for j = 1:size(rovers,2)
			if i~=j && rovers(j).destination == rovers(i).destination 
				error('Identical destinations');
			end
		end
	end

	if run_params.debugging_level > 0 
		fprintf('[info]destinations selected to all rovers\n');
	end

	time = 0;
	rover_replannings = zeros(size(rovers,2),2);
	tasks = zeros(size(rovers,2),1);
	lambdas = cell(size(rovers,2),1);
	phis = cell(size(rovers,2),1);
	alphas = zeros(size(rovers,2),1);
	for i = 1:size(rovers,2)
		tasks(i,1) = rovers(i).position;
		for j = 2:2:size(rovers(i).tasks,2)
			tasks(i,j/2+1) = rovers(i).tasks(j).goal;
		end
	end
	optimal_tasks = tasks;
	T = size(tasks,2)-1;
	for i = 1:size(rovers,2)
		clear last;
		for j = 1:size(tasks,2)
			if tasks(i,j) ~= 0
				last = tasks(i,j);
			else
				tasks(i,j) = last;
			end
		end
		lambdas{i}(1,path_size,T) = 0;
		phis{i}(1,path_size,path_size,T) = 0;
	end
	
	delta_lambda = run_params.delta_lambda;
	delta_phi = run_params.delta_phi;
	delta_alpha = run_params.delta_alpha;
	n = 1;
	rounds = 1;

	if collision(rovers,run_params)
		drawnow;
		if run_params.debugging_level > 0 
			fprintf('[info]collision found\n');
		end
		tic;
		subgradients = subgradients_generator(path_size,optimal_tasks,tasks,rovers);
		time_temp = toc;
		time = time + time_temp;
	end

	while  rounds <= 500 && ( collision(rovers,run_params) || isequal(rover_replannings(:,1),rover_replannings(:,2)) )
		tic;
		clear C;
		C = tentative_subgradients_generator(path_size,tasks,rovers);
		rover_replannings(:,2) = rover_replannings(:,1);

		if run_params.debugging_level > 0 
			fprintf('[info]round begin:%d\n',rounds);
		end

		for k = 1:size(rovers,2)
			for i = 1:path_size
				for t = 1:T
					if size(subgradients.lambda,3) < t
						subgradients = subgradients_generator(path_size,optimal_tasks,tasks,rovers,t);
					end
					%calculating lagrange multipliers
					if size(lambdas{k},3) >= t
						prev_lambda = lambdas{k}(n,i,t);
					else
						prev_lambda = 0;
					end
					lambdas{k}(n+1,i,t) = max(0,prev_lambda + delta_lambda * (subgradients.lambda(k,i,t)-1));
					for j = 1:size(rovers,2)
						if size(phis{k},4) >= t
							prev_phi = phis{k}(n,i,t);
						else
							prev_phi = 0;
						end
						phis{k}(n+1,i,j,t) = max(0,prev_phi + delta_phi * (subgradients.phi(k,i,j,t)-1));
					end
				end
			end
			%calculating the alphas
			alphas(k,n+1) = alphas(k,n) + delta_alpha * C{1}(k)+C{2}(k); 
		end

		alphas(:,n) = alphas(:,n+1);
		for k = 1:size(rovers,2)
			lambdas{k}(n,:,:) = lambdas{k}(n+1,:,:);
			phis{k}(n,:,:,:) = phis{k}(n+1,:,:,:);
		end

		clear d;
		rounds = rounds + 1;

		for k = 1:size(rovers,2)
			for i = 1:path_size
			for j = 1:path_size
			for t = 1:T
				sum = 0;
				for l = 1:size(rovers,2)
					if l ~= k 
						if tasks(l,t+1) == i
							sum = sum + 1;
						end
						if tasks(l,t+1) == i || tasks(l,t+1) == j
							sum = sum + 1;
						end	
					end
				end
				penalties(i,j,t) = 1 + alphas(k,n)*sum + lambdas{k}(n,j,t) + phis{k}(n,i,j,t) + phis{k}(n,j,i,t);
			end
			end
			end

			prob = round(10/run_params.replan_prob);

			rover_pg = dijkstra(pg,penalties,rovers(k).position,10);
			clear temp_tasks;
			temp_tasks = get_tasks_dijkstra(rover_pg, rovers(k).destination, rovers(k).pg_position);
			if size(temp_tasks,2) == size(rovers(k).tasks,2)
				tasks_changed = false;
				for j = 2:2:size(rovers(k).tasks,2)
					if rovers(k).tasks(j).goal ~= temp_tasks(j).goal
						rover_replannings(k,1) = rover_replannings(k,1) + 1;
						break;
					end
				end
			else
				rover_replannings(k,1) = rover_replannings(k,1) + 1;
			end

			if randi([0 prob]) < 10
				rovers(k).tasks = temp_tasks;
				if run_params.debugging_level > 0 
					fprintf('[info]rover(%d) trajectory replanned\n',k);
				end
			else
				if run_params.debugging_level > 0 
					fprintf('[info]rover(%d) trajectory not replanned\n',k);
				end
			end
		end
		tasks = zeros(size(rovers,2),1);
		for i = 1:size(rovers,2)
			tasks(i,1) = rovers(i).position;
			for j = 2:2:size(rovers(i).tasks,2)
				tasks(i,j/2+1) = rovers(i).tasks(j).goal;
			end
		end
		T = size(tasks,2)-1;
		for i = 1:size(rovers,2)
			clear last;
			for j = 1:size(tasks,2)
				if tasks(i,j) ~= 0
					last = tasks(i,j);
				else
					tasks(i,j) = last;
				end
			end
		end
		time_temp = toc;
		time = time + time_temp;
		for i = 1:size(rovers,2)
			%displaying rover lines
			clear line_x;
			clear line_y;
			if ~isempty(rover_handles(i).trajectory)
				rover_handles(i).trajectory.delete;
			end
			line_x(1) = vertices_pos(rovers(i).position,1);
			line_y(1) = vertices_pos(rovers(i).position,2);
			%changed for version with no rotation
			for j = 2:2:size(rovers(i).tasks,2)
				%str = PG.Nodes.Name(rovers(i).tasks(j).goal);
				%res = strsplit(str{1},'-');
				destination = rovers(i).tasks(j).goal;
				%destination = str2num(res{2});
				line_x(j/2+1) = vertices_pos(destination,1);
				line_y(j/2+1) = vertices_pos(destination,2);
			end
			hold on;
			rover_handles(i).trajectory = plot(line_x,line_y,colors(i));
			hold off;
			drawnow;
			if run_params.debugging_level > 0 
				fprintf('[gui]rover trajectory redrawed\n');
			end
			if run_params.stop_level > 2
				waitforbuttonpress;
			end
		end

	end
	
	res_success = true;
	results = struct('n',rounds,'rover_rounds',rover_replannings(:,1),'replan_time',time);
	if rounds >= 500 
		res_success = false;
		clear results;
		results = struct('n',rounds,'rover_rounds',rover_replannings(:,1),'replan_time',time,'alpha',alphas,'lambda',lambdas,'phi',phis);

		fprintf('heey');
	end

	ans = {rovers,results,res_success};
end

function ans = subgradients_generator(path_size,original,tasks,rovers,t)
	%calculating the subgradients
	if nargin == 4
		T = size(tasks,2)-1;
	else
		T = t;
	end
	subgradient_lambda = zeros(size(rovers,2),path_size,T);
	subgradient_lambda = subgradient_lambda - 1;
	for i = 1:path_size
		for t = 1:T
			for k = 1:size(rovers,2)
				if size(original,2) >= t && original(k,t) == i
					subgradient_lambda(k,i,t) = subgradient_lambda(k,i,t) + 1;
				end
				for l = 1:size(rovers,2)
					if l~=k && size(tasks,2) >= t && tasks(l,t) == i
						subgradient_lambda(k,i,t) = subgradient_lambda(k,i,t) + 1;
					end
				end
			end
		end
	end

	clear subgradient_phi;
	subgradient_phi(size(rovers,2),path_size,path_size,T) = 0;
	subgradient_phi = subgradient_phi - 1;
	
	for j = 1:path_size
		for i = 1:path_size	
			for k = 1:size(rovers,2)
				for t = 2:T
					if size(original,2) >= t && ((original(k,t) == i && original(k,t-1) == j) || (original(k,t) == j && original(k,t-1)) == i)
						subgradient_phi(k,i,j,t) = subgradient_phi(k,i,j,t) + 1;
					end
					for l = 1:size(rovers,2)
						if l ~= k && ((tasks(l,t) == i && tasks(l,t-1) == j) || (tasks(l,t) == j && tasks(l,t-1) == i)) 
							subgradient_phi(k,i,j,t) = subgradient_phi(k,i,j,t) + 1;
						end
					end
				end
			end
		end
	end	

	ans = struct('lambda',subgradient_lambda,'phi',subgradient_phi);
end

function ans = tentative_subgradients_generator(path_size,tasks,rovers)
	clear C1;
	clear C2;
	C1 = zeros(size(rovers,2),1);
	C2 = zeros(size(rovers,2),1);
	T = size(tasks,2) -1;

	for k = 1:size(rovers,2)
		for t = 1:T
			for l = 1:size(rovers,2)
				if l ~= k
					if tasks(k,t) == tasks(l,t)
						C1(k) = C1(k) + 1;
					end
				end
				if t ~=size(tasks,2) && l ~= k
					if tasks(k,t) == tasks(l,t+1) && tasks(k,t+1) == tasks(l,t)
						C2(k) = C2(k) + 1;
					end
				end
			end
		end
	end
	
	ans = {C1,C2};
end

function ans = collision(rovers,run_params)
	no_init_collision = false(size(rovers,2),1);
	for i = 1:size(rovers,2)
		for j = 2:2:size(rovers(i).tasks,2)
			for k = 1:size(rovers,2)
				if i ~= k
					if no_init_collision(i) == false && rovers(i).position == rovers(k).tasks(2).goal && rovers(k).position == rovers(i).tasks(2).goal %checking if there is a collision at the beginning
						if run_params.debugging_level > 5 
							fprintf('[info]beginning collision\n');
						end
						ans = true;
						return;
					else
						no_init_collision(i) = true;
					end
					if size(rovers(k).tasks,2) > j
						if rovers(i).tasks(j).goal == rovers(k).tasks(j).goal %two rovers are going to the same vertex
							if run_params.debugging_level > 5 
								fprintf('same vertex collision\n');
							end
							ans = true;
							return;
						end
						if j > 2 && rovers(i).tasks(j).goal == rovers(k).tasks(j-2).goal && rovers(i).tasks(j-2).goal == rovers(k).tasks(j).goal %rovers are trying to go on the same edge
							if run_params.debugging_level > 5 
								fprintf('same node collision\n');
							end
							ans = true;
							return;
						end
					else
						if rovers(i).tasks(j).goal == rovers(k).tasks(size(rovers(k).tasks,2)-1).goal %if we are crossing a steady rover
							if run_params.debugging_level > 5 
								fprintf('steady rover collision\n');
							end
							ans = true;
							return;
						end
					end
				end
			end
		end
	end
	ans = false;
end

function ans = do(map,rover_datas)
	load(map);
	load(rover_datas);
	path_size = parameters.path_height*parameters.path_width;

	pg = convert(adjacency_mtx);
	pg_f = floyd(pg);

	%displaying rovers
	for i = 1:size(rovers,2)
		rover_handles(i) = struct( 'rover_patch', [], 'trajectory' , []);

		x_centered = rovers(i).x+x;
		y_centered = rovers(i).y+y;
		x_done = [1:size(x,2)];
		y_done = [1:size(x,2)];

		for j = 1:size(x_done,2)
			x_done(j) = x(j)*cos(rovers(i).rotation)-y(j)*sin(rovers(i).rotation);
		end
		for j = 1:size(y_done,2)
			y_done(j) = x(j)*sin(rovers(i).rotation)+y(j)*cos(rovers(i).rotation);
		end
		x_centered = x_done + rovers(i).x;
		y_centered = y_done + rovers(i).y;

		rover_handles(i).rover_patch = patch(x_centered,y_centered,colors(i));
		if run_params.debugging_level > 6
			fprintf('[gui]rover[%d] displayed\n',i);
		end
	end

	clear global rover_handles;
	global rover_handles;
	results = init_destinations(rovers,G,map,rover_datas,run_params);
	rovers = results{1};

	simulation_running = true;

	for i = 1:size(rovers,2)
		rover_steps(i) = (size(rovers(i).tasks,2) - 1)/2;
	end

	while simulation_running && run_params.animate
		for i = 1:size(rovers,2)
			if run_params.debugging_level > 7 
				fprintf('[info]rover:%d\n',i);
			end
			if rovers(i).tasks(1).type == movement.nope
				if run_params.debugging_level > 7 
					fprintf('[type:]nope\n');
				end
				if ~isnan(rovers(i).pg_position)
				end
				if run_params.stop_at_end 
					all_stopped = true;
					for j = 1:size(rovers,2)
						if rovers(j).tasks(1).type ~= movement.nope
							all_stopped = false;
						end
					end
					if all_stopped
						simulation_running = false;
					end
				end
			end
			if rovers(i).tasks(1).type == movement.moving
				if run_params.debugging_level > 7 
					fprintf('[type:]moving\n');
				end
				%str = PG.Nodes.Name(rovers(i).tasks(1).goal);
				%res = strsplit(str{1},'-');
				%destination = str2num(res{2});
				destination = rovers(i).tasks(1).goal;
				if rovers(i).tasks(1).time < run_params.time_scale
					%rovers(i).reached the destination node
					if run_params.debugging_level > 0 
						fprintf('[event]rover[%d] arrived to:%d\n',i,destination);
					end
					reserved_vertices(rovers(i).position) = false;
					rovers(i).reserved = NaN;
					rovers(i).position = destination;
					rovers(i).pg_position = rovers(i).tasks(1).goal;
					rovers(i).x = vertices_pos(rovers(i).position,1);
					rovers(i).y = vertices_pos(rovers(i).position,2);
					rovers(i).tasks(1) = [];
					if run_params.debugging_level > 8 
						fprintf('[variable]pg_position:%d\n',rovers(i).pg_position);
					end

					for j = 1:size(x_done,2)
						x_done(j) = x(j)*cos(rovers(i).rotation)-y(j)*sin(rovers(i).rotation);
					end
					for j = 1:size(y_done,2)
						y_done(j) = x(j)*sin(rovers(i).rotation)+y(j)*cos(rovers(i).rotation);
					end
					x_centered = x_done + rovers(i).x;
					y_centered = y_done + rovers(i).y;
					set(rover_handles(i).rover_patch,'XData',x_centered,'YData',y_centered);
					drawnow;
				else
					%rovers(i).still moves on te line vetween two verteces
					if run_params.debugging_level > 0 
						fprintf('[event]rover[%d] moving to:%d\n',i,destination);
					end
					rovers(i).tasks(1).time = rovers(i).tasks(1).time - run_params.time_scale;
					vector = [ vertices_pos(destination,1)-rovers(i).x vertices_pos(destination,2)-rovers(i).y ];
					if run_params.debugging_level > 8 
						fprintf('[variable]vector parameters:%f:%f\n',vector(1), vector(2));
					end
					rovers(i).x = rovers(i).x + vector(1)*(run_params.time_scale);
					rovers(i).y = rovers(i).y + vector(2)*(run_params.time_scale);
					for j = 1:size(x_done,2)
						x_done(j) = x(j)*cos(rovers(i).rotation)-y(j)*sin(rovers(i).rotation);
					end
					for j = 1:size(y_done,2)
						y_done(j) = x(j)*sin(rovers(i).rotation)+y(j)*cos(rovers(i).rotation);
					end
					x_centered = x_done + rovers(i).x;
					y_centered = y_done + rovers(i).y;
					set(rover_handles(i).rover_patch,'XData',x_centered,'YData',y_centered);
					drawnow;
				end
			end
			if rovers(i).tasks(1).type == movement.rotating
				if run_params.debugging_level > 7
					fprintf('[type:]rotating\n');
				end
				%checking if the destination is not reserved
				enabled = false;
				%str = PG.Nodes.Name(rovers(i).tasks(2).goal);
				%res = strsplit(str{1},'-');
				destination = rovers(i).tasks(2).goal;
				%destination = str2num(res{2});
				%if ~isnan(rovers(i).reserved)
				%	enabled = true;
				%else 
				%	if reserved_vertices( destination ) == false
				%		reserved_vertices( destination ) = true;
				%		rovers(i).reserved = destination;
				%		enabled = true;
				%	end
				%end
				rovers(i).tasks(1) = [];
				if enabled
					if abs(rovers(i).tasks(1).goal) < run_params.time_scale
						%Rover is heading to the requested
						rovers(i).rotation = rovers(i).rotation + rovers(i).tasks(1).goal;
						rovers(i).tasks(1) = [];
						for j = 1:size(x_done,2)
							x_done(j) = x(j)*cos(rovers(i).rotation)-y(j)*sin(rovers(i).rotation);
						end
						for j = 1:size(y_done,2)
							y_done(j) = x(j)*sin(rovers(i).rotation)+y(j)*cos(rovers(i).rotation);
						end
						x_centered = x_done + rovers(i).x;
						y_centered = y_done + rovers(i).y;
						set(rover_handles(i).rover_patch,'XData',x_centered,'YData',y_centered);
						drawnow;
					else
						if abs(rovers(i).tasks(1).goal) == rovers(i).tasks(1).goal
							rovers(i).rotation = rovers(i).rotation + run_params.time_scale;
							rovers(i).tasks(1).goal = rovers(i).tasks(1).goal - run_params.time_scale;
						else
							rovers(i).rotation = rovers(i).rotation - run_params.time_scale;
							rovers(i).tasks(1).goal = rovers(i).tasks(1).goal + run_params.time_scale;
						end

						for j = 1:size(x_done,2)
							x_done(j) = x(j)*cos(rovers(i).rotation)-y(j)*sin(rovers(i).rotation);
						end
						for j = 1:size(y_done,2)
							y_done(j) = x(j)*sin(rovers(i).rotation)+y(j)*cos(rovers(i).rotation);
						end
						x_centered = x_done + rovers(i).x;
						y_centered = y_done + rovers(i).y;
						set(rover_handles(i).rover_patch,'XData',x_centered,'YData',y_centered);
						drawnow;
					end
				else
					continue;
					%using a timer to avoid deadlocks
					if run_params.debugging_level > 0
						fprintf('[event]rover[%d] collision\n',i);
					end
					if rovers(i).timeout_counter > run_params.timeout
						if run_params.debugging_level > 0 
							fprintf('[event]timeout reached\n');
						end
						
						rovers(i).tasks(1).type = movement.nope;
						rovers(i).timeout_counter = 0;
					else
						rovers(i).timeout_counter = rovers(i).timeout_counter + 1;
					end
				end	
			end
		end
		if run_params.stop_level > 6
			fprintf('holding until key press\n');
			waitforbuttonpress;
		end
	end

	if results{3} == true
		ans = struct('success',results{3},'result_parameters', results{2},'rover_steps',rover_steps);
	else
		ans = struct('success',results{3},'result_parameters', results{2});
	end

end
