function test(delta_alpha,delta_lambda,delta_phi,prob,name)
	rng('shuffle');
	roverd = RunParameters();
	roverd.rover_positions =  [1 8 62 134 123 126 50];
	roverd.rover_destinations = [86 146 99 22 26 14 143];
	roverd.rovers_num = 7;
	roverd.animate = false;
	log = fopen('test.txt','a');
	export = fopen(num2str(name),'a');
	roverd.delta_alpha = delta_alpha*0.01;
	roverd.delta_lambda = delta_lambda*0.01;
	roverd.delta_phi = delta_phi*0.01;
	roverd.replan_prob = prob / 100;

	fprintf(log,'**********************************\ndelta_alpha=%d delta_lambda=%d delta_phi=%d replan_prob=%d\n',roverd.delta_alpha,roverd.delta_lambda,roverd.delta_phi,roverd.replan_prob);
	fprintf(export,'%d\t%d\t%d\t%d\t',roverd.delta_alpha,roverd.delta_lambda,roverd.delta_phi,roverd.replan_prob);

	res = run('original',roverd);
	if res.success
		fprintf(log,'-----------------------------\n');
		fprintf(log,'n=%d\nsum=%d\ntime=%d\n',res.result_parameters.n,sum(res.rover_steps),res.result_parameters.replan_time);
		fprintf(export,'%d\t%d\t%d\t',res.result_parameters.n,sum(res.rover_steps),res.result_parameters.replan_time);
		for j = 1:7
			fprintf(export,'%d\t',res.rover_steps(j));
			fprintf(log,'rover[%d]=%d ',j,res.rover_steps(j));
		end
		fprintf(log,'\n replannings:\n');
		for j = 1:7
			fprintf(export,'%d\t',res.result_parameters.rover_rounds(j));
			fprintf(log,'rover[%d]=%d',j,res.result_parameters.rover_rounds(j));
		end
		fprintf(log,'\n\n');
		fprintf(export,'\n');
	else
		fprintf(log,'-----------------------------\n');
		fprintf(log,'error, round limit reached');
		num = randi([0 1000]);
		fprintf(export,'error timeout ID:%d',num);
	
		res.result_parameters.alpha
		res.result_parameters.lambda
		res.result_parameters.phi

		%dlmwrite(num2str(num),res.result_parameters.alpha);
		%dlmwrite(num2str(num),res.result_parameters.lambda);
		%dlmwrite(num2str(num),res.result_parameters.phi);

		fprintf(log,'\n\n');
		fprintf(export,'\n');
	end

	fclose(log);
	fclose(export);
	exit;
end
