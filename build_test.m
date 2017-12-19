%function ans=build(parameters)
	%if no parameters was given, generating parameters:
%	if nargin == 0
		parameters = MapParameters();
%	end
	%generating graph according to the given data
	%results:
	%% *adjacency_mtx - the adjacency matrix of the graph
	%% *vertices_pos - the positions of the nodes
	%% *G - the graph object for the display
	[vertices_pos_x vertices_pos_y] = meshgrid(1:parameters.path_width,1:parameters.path_height);

	path_size = parameters.path_width*parameters.path_height;
	vertices_pos = [ vertices_pos_x(:) vertices_pos_y(:) ];
	adjacency_mtx = zeros(path_size, path_size);

	%generationg random edges in the matrix
	edge_prob = floor(parameters.edge_probability / ( 1 - parameters.edge_probability));
	for i = 1:path_size
		for j = [-1 1]
			%horizontal verices
			if i+j <= path_size && i+j > 0 && ceil((i+j)/parameters.path_height) == ceil(i/parameters.path_height)
				adjacency_mtx(i+j,i) = randi([0 edge_prob]);
				if adjacency_mtx(i+j,i) >= 1
					adjacency_mtx(i+j,i) = parameters.edge_weight;
				end
			end

			%vertical vertices
			if i+(j*parameters.path_height) <= path_size && i+(j*parameters.path_height) > 0 
				adjacency_mtx(i,i+(j*parameters.path_height)) = randi([0 edge_prob]);
				if adjacency_mtx(i,i+(j*parameters.path_height)) >= 1
					adjacency_mtx(i,i+(j*parameters.path_height)) = parameters.edge_weight;
				end
			end
		end
	end

	%converting the adjacency matrix to symmetrix
	for i = 1:path_size
		for j = i:path_size
			adjacency_mtx(i,j) = adjacency_mtx(j,i);
		end
	end

	%setting names to the Nodes for the display
	names = strings;
	for i = 1:path_size
		names(i,1) = num2str(i);
	end

	%loadong matrix data to the graph class
	G = graph(adjacency_mtx);
	G.Nodes.Names = names;

%creating planning and mapping graph
%results:
%% *PG graph object to display
	PG = digraph();
	n = size(G.Edges.EndNodes,1);
	%adding node to the planning graph
	for j = 1:n
		%the name: the second number is the node to arrive from the first node
		PG = PG.addnode(strcat(strcat(num2str(G.Edges.EndNodes(j,1)),'-'),num2str(G.Edges.EndNodes(j,2))));
		PG = PG.addnode(strcat(strcat(num2str(G.Edges.EndNodes(j,2)),'-'),num2str(G.Edges.EndNodes(j,1))));
	end
	
	pg_adjacency = Inf(size(G.Edges.EndNodes,1)*2);
	indexing_mtx = Inf(path_size);
	pg_edges = zeros(size(pg_adjacency,1),2);

	for i = 1:size(pg_adjacency,1)
		if mod(i,2) == 0
			node_source = G.Edges.EndNodes(ceil(i/2),2);
			node_destination = G.Edges.EndNodes(ceil(i/2),1);
		else
			node_source = G.Edges.EndNodes(ceil(i/2),1);
			node_destination = G.Edges.EndNodes(ceil(i/2),2);
		end

		indexing_mtx(node_source,node_destination) = i;
	end

	for i = 1:size(pg_adjacency,1)
		if mod(i,2) == 0
			node_source = G.Edges.EndNodes(ceil(i/2),2);
			node_destination = G.Edges.EndNodes(ceil(i/2),1);
		else
			node_source = G.Edges.EndNodes(ceil(i/2),1);
			node_destination = G.Edges.EndNodes(ceil(i/2),2);
		end
		pg_edges(i,1) = node_source;
		pg_edges(i,2) = node_destination;

		for j = [-1 1]
			node_next = node_destination + 1;
			%right neighbour
			if node_next <= path_size && ceil(node_next/parameters.path_height) == ceil(node_next/parameters.path_height)
				%checking if neighbour is connected
				if adjacency_mtx(node_destination,node_next) ~= 0
					%selecting the previous -point_0- node and the current -point_1- node to calculate the rotation needed to reach the nextnode -point_2-
					point_0 = [ vertices_pos(node_source,1) vertices_pos(node_source,2) ];
					point_1 = [ vertices_pos(node_destination,1) vertices_pos(node_destination,2) ];
					point_2 = [ vertices_pos(node_next,1) vertices_pos(node_next,2) ];
					pg_path = indexing_mtx(node_destination,node_next);
					rotation_cost = abs(wrapToPi(atan2(point_2(2)-point_1(2), point_2(1)-point_1(1)) - atan2(point_1(2)-point_0(2),point_1(1)-point_0(1))));
					pg_adjacency(i,pg_path) = adjacency_mtx(node_destination,node_next) + rotation_cost;
				end
			end

			node_next = node_destination + (j*parameters.path_height);
			%upper neighbour
			if node_next <= path_size && node_next > 0 
				%checking if neighbour is connected
				if adjacency_mtx(node_destination,node_next) ~= 0
					%selecting the previous -point_0- node and the current -point_1- node to calculate the rotation needed to reach the nextnode -point_2-
					point_0 = [ vertices_pos(node_source,1) vertices_pos(node_source,2) ];
					point_1 = [ vertices_pos(node_destination,1) vertices_pos(node_destination,2) ];
					point_2 = [ vertices_pos(node_next,1) vertices_pos(node_next,2) ];
					pg_path = indexing_mtx(node_destination,node_next);
					rotation_cost = abs(wrapToPi(atan2(point_2(2)-point_1(2), point_2(1)-point_1(1)) - atan2(point_1(2)-point_0(2),point_1(1)-point_0(1))));
					pg_adjacency(i,pg_path) = adjacency_mtx(node_destination,node_next) + rotation_cost;
				end
			end

			node_next = node_destination -1;
			%left neighbour
			if node_next <= path_size && node_next > 0 && ceil(node_next/parameters.path_height) == ceil(node_next/parameters.path_height)
				%checking if neighbour is connected
				if adjacency_mtx(node_destination,node_next) ~= 0
					%selecting the previous -point_0- node and the current -point_1- node to calculate the rotation needed to reach the nextnode -point_2-
					point_0 = [ vertices_pos(node_source,1) vertices_pos(node_source,2) ];
					point_1 = [ vertices_pos(node_destination,1) vertices_pos(node_destination,2) ];
					point_2 = [ vertices_pos(node_next,1) vertices_pos(node_next,2) ];
					pg_path = indexing_mtx(node_destination,node_next);
					rotation_cost = abs(wrapToPi(atan2(point_2(2)-point_1(2), point_2(1)-point_1(1)) - atan2(point_1(2)-point_0(2),point_1(1)-point_0(1))));
					pg_adjacency(i,pg_path) = adjacency_mtx(node_destination,node_next) + rotation_cost;
				end
			end

			node_next = node_destination - (j*parameters.path_height);
			%bootom neighbour
			if node_next <= path_size && node_next > 0 
				%checking if neighbour is connected
				if adjacency_mtx(node_destination,node_next) ~= 0
					%selecting the previous -point_0- node and the current -point_1- node to calculate the rotation needed to reach the nextnode -point_2-
					point_0 = [ vertices_pos(node_source,1) vertices_pos(node_source,2) ];
					point_1 = [ vertices_pos(node_destination,1) vertices_pos(node_destination,2) ];
					point_2 = [ vertices_pos(node_next,1) vertices_pos(node_next,2) ];
					pg_path = indexing_mtx(node_destination,node_next);
					rotation_cost = abs(wrapToPi(atan2(point_2(2)-point_1(2), point_2(1)-point_1(1)) - atan2(point_1(2)-point_0(2),point_1(1)-point_0(1))));
					pg_adjacency(i,pg_path) = adjacency_mtx(node_destination,node_next) + rotation_cost;
				end
			end
		end
	end


	%adding the edges to the planning graph
	n = size(PG.Nodes,1);
	for i = 1:n
		str = PG.Nodes.Name(i);
		res = strsplit(str{1},'-');
		for j = [-1 1]
			%%id_p: the node from where the rover comes
			%%id_s: the node where the rover arrives
			id_p = str2num(res{1});
			id_s = str2num(res{2});

			%%id_d: neighbours if the id_s which can be connected with id_s
			id_d = str2num(res{2})+j;
			%right neighbour
			if id_d <= path_size && id_d > 0 && ceil(id_d/parameters.path_height) == ceil(id_s/parameters.path_height)
				if adjacency_mtx(id_s,id_d) ~= 0 %if there is a connection
					%selecting the previous -point_0- node and the current -point_1- node to calculate the rotation needed to reach the next node -point_2-
					point_0 = [ vertices_pos(id_p,1) vertices_pos(id_p,2) ];
					point_1 = [ vertices_pos(id_s,1) vertices_pos(id_s,2) ];
					point_2 = [ vertices_pos(id_d,1) vertices_pos(id_d,2) ];
					id_s_pg = PG.findnode(strcat(strcat(num2str(id_p),'-'),num2str(id_s)));
					id_d_pg = PG.findnode(strcat(strcat(num2str(id_s),'-'),num2str(id_d)));
					rotation_cost = abs(wrapToPi(atan2(point_2(2)-point_1(2), point_2(1)-point_1(1)) - atan2(point_1(2)-point_0(2),point_1(1)-point_0(1))));
					if ~PG.findedge(id_s_pg,id_d_pg)
						PG = PG.addedge(id_s_pg,id_d_pg,adjacency_mtx(id_s,id_d)+rotation_cost);
					end
				end
			end

			id_d = str2num(res{2})+(j*parameters.path_height);
			%upper neighbour
			if id_d <= path_size && id_d > 0 
				if adjacency_mtx(id_s,id_d) ~= 0 %if there is a connection
					point_0 = [ vertices_pos(id_p,1) vertices_pos(id_p,2) ];
					point_1 = [ vertices_pos(id_s,1) vertices_pos(id_s,2) ];
					point_2 = [ vertices_pos(id_d,1) vertices_pos(id_d,2) ];
					id_s_pg = PG.findnode(strcat(strcat(num2str(id_p),'-'),num2str(id_s)));
					id_d_pg = PG.findnode(strcat(strcat(num2str(id_s),'-'),num2str(id_d)));
					rotation_cost = abs(wrapToPi(atan2(point_2(2)-point_1(2), point_2(1)-point_1(1)) - atan2(point_1(2)-point_0(2),point_1(1)-point_0(1))));
					if ~PG.findedge(id_s_pg,id_d_pg)
						PG = PG.addedge(id_s_pg,id_d_pg,adjacency_mtx(id_s,id_d)+rotation_cost);
					end
				end
			end

			id_d = str2num(res{1})+j;
			%left neighbour
			if id_d <= path_size && id_d > 0 && ceil(id_d/parameters.path_height) == ceil(id_s/parameters.path_height)
				if adjacency_mtx(id_s,id_d) ~= 0 %if there is a connection
					point_0 = [ vertices_pos(id_p,1) vertices_pos(id_p,2) ];
					point_1 = [ vertices_pos(id_s,1) vertices_pos(id_s,2) ];
					point_2 = [ vertices_pos(id_d,1) vertices_pos(id_d,2) ];
					id_s_pg = PG.findnode(strcat(strcat(num2str(id_p),'-'),num2str(id_s)));
					id_d_pg = PG.findnode(strcat(strcat(num2str(id_s),'-'),num2str(id_d)));
					rotation_cost = abs(wrapToPi(atan2(point_2(2)-point_1(2), point_2(1)-point_1(1)) - atan2(point_1(2)-point_0(2),point_1(1)-point_0(1))));
					if ~PG.findedge(id_s_pg,id_d_pg)
						PG = PG.addedge(id_s_pg,id_d_pg,adjacency_mtx(id_s,id_d)+rotation_cost);
					end
				end
			end

			id_d = str2num(res{2})+(j*parameters.path_height);
			%bottom neighbour
			if id_d <= path_size && id_d > 0
				if adjacency_mtx(id_s,id_d) ~= 0 %if there is a connection
					point_0 = [ vertices_pos(id_p,1) vertices_pos(id_p,2) ];
					point_1 = [ vertices_pos(id_s,1) vertices_pos(id_s,2) ];
					point_2 = [ vertices_pos(id_d,1) vertices_pos(id_d,2) ];
					id_s_pg = PG.findnode(strcat(strcat(num2str(id_p),'-'),num2str(id_s)));
					id_d_pg = PG.findnode(strcat(strcat(num2str(id_s),'-'),num2str(id_d)));
					rotation_cost = abs(wrapToPi(atan2(point_2(2)-point_1(2), point_2(1)-point_1(1)) - atan2(point_1(2)-point_0(2),point_1(1)-point_0(1))));
					if ~PG.findedge(id_s_pg,id_d_pg)
						PG = PG.addedge(id_s_pg,id_d_pg,adjacency_mtx(id_s,id_d)+rotation_cost);
					end
				end
			end
		end
	end

	pg = convert(PG);
	pg_f = floyd(pg);
	save 'path' 'PG' 'G' 'adjacency_mtx' 'vertices_pos' 'parameters' 'pg_f'
	ans = 'path';

%end
