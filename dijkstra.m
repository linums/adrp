function ans = dijkstra(adjacency_mtx,G,source,multiplier)

	if size(G,1) ~= size(G,2) || size(adjacency_mtx,1) ~= size(adjacency_mtx,2) || size(adjacency_mtx,1) ~= size(G,2)
		error('Dijkstra function was called not with a n x n size adjacency matrix of a graph');
	end

	n = size(G,1);
	dist = Inf(n,1);
	prev = NaN(n,1);
	time_dist = NaN(n,1);
	dist(source) = 0;
	Q = (1:n);
	time_dist(source) = 0;

	while size(Q,2) ~= 0
		lowest = NaN;
		lo_dist = Inf;
		for i = 1:n
			contains = false;
			for j = 1:size(Q,2)
				if Q(j) == i
					contains = true;
					break;
				end
			end
			if contains == true && lo_dist > dist(i)
				lowest = i;
				lo_dist = dist(i);
			end
		end

		for j = 1:size(Q,2)
			if Q(j) == lowest
				Q(j) = [];
				break;
			end
		end

		if isnan(lowest) %no more vertices in the Q within finite distance
			break;
		end

		for i = 1:n
			if ~isinf(adjacency_mtx(lowest,i)) 
				if size(G,3) < time_dist(lowest) || time_dist(lowest) == 0
					penalty = 0;
				else 
					penalty = G(lowest,i,time_dist(lowest))+1;
				end

			        if (penalty * multiplier + dist(lowest))  < dist(i)
					time_dist(i) = time_dist(lowest)+1;
					dist(i) = penalty * multiplier + dist(lowest);% + adjacency_mtx(lowest,i);
					prev(i) = lowest;
				end
			end
		end


	end

	ans = convert_to_compatibility(dist,prev,source);

end

function ans = convert_to_compatibility(dist,res,source)
	
	ans = {dist,res};
	return;	

	n = size(dist,1);
	G = Inf(n);
	route = strings(n);
	for i = 1:size(dist,1)
		G(source,i) = dist(i);
		if isnan(res(i))
			route(source,i) = '';
		else
			route(source,i) = num2str(res(i));
		end
	end
	ans = {G,route};
end
