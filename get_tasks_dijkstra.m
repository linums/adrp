function ans = get_tasks_dijkstra(pg, dest, source)
	endt = Task();
	res = get_tasks_main(pg, dest, source);
	ans = [res endt];
end

function ans= get_tasks_main(pg, dest, source)
	if ~isnan(pg{2}(dest))
			mov = Task();
			mov.type = 'moving';
			mov.goal = dest;
			mov.time = 10;%pg{1}(source,dest);
			rot = Task();
			rot.type = 'rotating';
			%rot.goal = get_rotation( x_y, source, dest, PG );
			rot.goal = 0;
			rot.time = 0;
			%rot.time = abs(rot.goal);
			ans = [rot mov];
		if pg{2}(dest) == source
			ans = [rot mov];
		else
			ans = [get_tasks_main(pg,pg{2}(dest),source) rot mov];
		end
	end
end


function ans = get_rotation( x_y, src, dest, pg )
	src_str = pg.Nodes.Name(src);
	src_res = strsplit(src_str{1},'-');
	dst_str = pg.Nodes.Name(dest);
	dst_res = strsplit(dst_str{1},'-');
	
	point_0 = [ x_y(str2num(src_res{1}),1)  x_y(str2num(src_res{1}),2) ];
	point_1 = [ x_y(str2num(src_res{2}),1)  x_y(str2num(src_res{2}),2) ];
	point_2 = [ x_y(str2num(dst_res{2}),1)  x_y(str2num(dst_res{2}),2) ];

	ans = wrapToPi(atan2(point_2(2)-point_1(2), point_2(1)-point_1(1)) - atan2(point_1(2)-point_0(2),point_1(1)-point_0(1)));
end
			
		
