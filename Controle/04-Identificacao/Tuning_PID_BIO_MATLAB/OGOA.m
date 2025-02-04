%_________________________________________________________________________%
%  Grasshopper Optimization Algorithm (GOA) source codes demo V1.0        %
%                                                                         %
%  Developed in MATLAB R2016a                                             %
%                                                                         %
%  Author and programmer: Seyedali Mirjalili                              %
%                                                                         %
%         e-Mail: ali.mirjalili@gmail.com                                 %
%                 seyedali.mirjalili@griffithuni.edu.au                   %
%                                                                         %
%       Homepage: http://www.alimirjalili.com                             %
%                                                                         %
%  Main paper: S. Saremi, S. Mirjalili, A. Lewis                          %
%              Grasshopper Optimisation Algorithm: Theory and Application %
%               Advances in Engineering Software , in press,              %
%               DOI: http://dx.doi.org/10.1016/j.advengsoft.2017.01.004   %
%                                                                         %
%_________________________________________________________________________%

% The Grasshopper Optimization Algorithm
function [globalmin,TargetPosition]=OGOA(N, Max_iter, lb,ub, dim,sp)
    
                    

disp('OGOA is now estimating the global optimum for your problem....')

flag=0;
if size(ub,1)==1
    ub=ones(dim,1)*ub;
    lb=ones(dim,1)*lb;
end

if (rem(dim,2)~=0) % this algorithm should be run with a even number of variables. This line is to handle odd number of variables
    dim = dim+1;
    ub = [ub; 100];
    lb = [lb; -100];
    flag=1;
end

%Initialize the population of grasshoppers
GrassHopperPositions=initializationGOA(N,dim,ub,lb);
GrassHopperFitness = zeros(1,N);

fitness_history=zeros(N,Max_iter);
position_history=zeros(N,Max_iter,dim);
Convergence_curve=zeros(1,Max_iter);
Trajectories=zeros(N,Max_iter);

cMax=1;
cMin=0.00004;
%Calculate the fitness of initial grasshoppers

for i=1:size(GrassHopperPositions,1)
    if flag == 1
        GrassHopperFitness(1,i)=tracklsq(GrassHopperPositions(i,1:end-1),sp);
    else
        GrassHopperFitness(1,i)=tracklsq(GrassHopperPositions(i,:),sp);
    end
    fitness_history(i,1)=GrassHopperFitness(1,i);
    position_history(i,1,:)=GrassHopperPositions(i,:);
    Trajectories(:,1)=GrassHopperPositions(:,1);
end

[sorted_fitness,sorted_indexes]=sort(GrassHopperFitness);

% Find the best grasshopper (target) in the first population 
for newindex=1:N
    Sorted_grasshopper(newindex,:)=GrassHopperPositions(sorted_indexes(newindex),:);
end

TargetPosition=Sorted_grasshopper(1,:);
TargetFitness=sorted_fitness(1);

% Main loop
l=2; % Start from the second iteration since the first iteration was dedicated to calculating the fitness of antlions
cnt_obl = 0; 
minFit = 0.1;
maxFNC = 30;
f_impr = 1e10;
FNC=0;
opposite=0;
while l<Max_iter+1
    
    c=cMax-l*((cMax-cMin)/Max_iter); % Eq. (2.8) in the paper
    
    for i=1:size(GrassHopperPositions,1)
        temp= GrassHopperPositions';
        for k=1:2:dim
            S_i=zeros(2,1);
            for j=1:N
                if i~=j
                    Dist=distance(temp(k:k+1,j), temp(k:k+1,i)); % Calculate the distance between two grasshoppers
                    
                    r_ij_vec=(temp(k:k+1,j)-temp(k:k+1,i))/(Dist+eps); % xj-xi/dij in Eq. (2.7)
                    xj_xi=2+rem(Dist,2); % |xjd - xid| in Eq. (2.7) 
                    
                    s_ij=((ub(k:k+1) - lb(k:k+1))*c/2)*S_func(xj_xi).*r_ij_vec; % The first part inside the big bracket in Eq. (2.7)
                    S_i=S_i+s_ij;
                end
            end
            S_i_total(k:k+1, :) = S_i;
            
        end
        
        X_new = c * S_i_total'+ (TargetPosition); % Eq. (2.7) in the paper      
        if opposite==0
            GrassHopperPositions_temp(i,:)=X_new'; 
        else
            opposite=0;
            mchg=rand();
            if rand()>0.5 
                GrassHopperPositions_temp(i,:)=-X_new'+ones(1,2)'*(rand-0.5)*0.1+ub;
            end 
        end
        
    end
    % GrassHopperPositions
    GrassHopperPositions=GrassHopperPositions_temp;
    
    for i=1:size(GrassHopperPositions,1)
        % Relocate grasshoppers that go outside the search space 
        Tp=GrassHopperPositions(i,:)>ub';Tm=GrassHopperPositions(i,:)<lb';GrassHopperPositions(i,:)=(GrassHopperPositions(i,:).*(~(Tp+Tm)))+ub'.*Tp+lb'.*Tm;
        
        % Calculating the objective values for all grasshoppers
        if flag == 1
            GrassHopperFitness(1,i)=tracklsq(GrassHopperPositions(i,1:end-1),sp);
        else
            GrassHopperFitness(1,i)=tracklsq(GrassHopperPositions(i,:),sp);
        end
        fitness_history(i,l)=GrassHopperFitness(1,i);
        position_history(i,l,:)=GrassHopperPositions(i,:);
        
        Trajectories(:,l)=GrassHopperPositions(:,1);
        
        % Update the target
        if GrassHopperFitness(1,i)<TargetFitness
            TargetPosition=GrassHopperPositions(i,:);
            TargetFitness=GrassHopperFitness(1,i);
        end
    end
    
    if (TargetFitness < minFit || TargetFitness < f_impr)
                        f_impr = TargetFitness;
                        FNC=0;
                        opposite=0;
                    elseif FNC == maxFNC
                        f_impr = 1e10;
                        FNC=0;
                        opposite=1;
                        cnt_obl = cnt_obl+1;
                    else
                        FNC=FNC+1;
                        opposite=0;
    end
                    
    Convergence_curve(l)=TargetFitness;
    %disp(['In iteration #', num2str(l), ' , target''s objective = ', num2str(TargetFitness)])
    
    l = l + 1;
    fprintf("Iteration ==> %d\n",l)
end
globalmin = Convergence_curve(Max_iter)

if (flag==1)
    TargetPosition = TargetPosition(1:dim-1);
end


