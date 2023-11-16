
global Jbest lambda_alpha_best lambda_slack_best lambda_alpha lambda_slack
vx = 80/3.6
Tsim = 30
fun = @(x) cost(x(1),x(2))


%%
Jbest = 100000


lambda_alpha_i = (logspace(-2, 3,10))
lambda_slack_i = (logspace(5, 9,10))
%%
lambda_alpha_best = lambda_alpha_i(1);
lambda_slack_best = lambda_slack_i(1);
Jij = zeros(10,10);
for i = 1:length(lambda_alpha_i)
    for j = 1:length(lambda_slack_i)
        J = cost(lambda_alpha_i(i),lambda_slack_i(j))
        if (J<Jbest)
            Jbest = J
            lambda_alpha_best  = lambda_alpha;
            lambda_slack_best = lambda_slack;
            Jij(i,j) = J;
            [i j]
            save rez.mat
        end
    end
end

%%

RunSimulation
Tsim = 30
vx = 80/3.6
%cost(11,1000)
%cost(11,00)
