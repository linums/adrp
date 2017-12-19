function ans = floyd(D)

	G = D;
	n = size(G, 1);	
	route = strings(n);

	for k = 1:n
		for i = 1:n
			for j = 1:n
				if G(i,j) > G(i,k) + G(k,j)
					route(i,j) = num2str(k);
					G(i,j) = G(i,k) + G(k,j);
				end
			end
		end
	end

	ans = {G,route};
end
