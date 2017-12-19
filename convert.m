function ans = convert(G)
	if isa(G,'graph') || isa(G,'digraph')
		num = size(G.Edges,1);

		adj = Inf(size(G.Nodes,1));

		for i = 1:num
			if G.Edges(i,2).Weight ~= 0
				adj(G.findnode(G.Edges(i,1).EndNodes(1)),G.findnode(G.Edges(i,1).EndNodes(2))) = G.Edges(i,2).Weight;
			else
				adj(G.findnode(G.Edges(i,1).EndNodes(1)),G.findnode(G.Edges(i,1).EndNodes(2))) = Inf;
			end
		end
	else
		[max_a,max_b] = size(G);

		max = max_a * max_b;

		for i = 1:max
			if G(i) == 0
				G(i) = Inf;
			end
		end

		adj = G;
	end

	ans = adj;
end
