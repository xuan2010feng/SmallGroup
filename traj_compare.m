function z=traj_compare(traj_a,traj_b)
    trajectorydistance=0;
    smallestdistance=10000;
    for i=1:size(traj_a,1)
        for j=1:size(traj_b,1)
               if E_distance(traj_a(i,1),traj_a(i,2),traj_b(j,1),traj_b(j,2))<smallestdistance
                smallestdistance=E_distance(traj_a(i,1),traj_a(i,2),traj_b(j,1),traj_b(j,2));
            end
        end
        trajectorydistance=trajectorydistance+smallestdistance;
        smallestdistance=10000;
    end
    %num计算轨迹中的nan数值，并排除掉
     num=numel(traj_a(isnan(traj_a)))/2;
     z= trajectorydistance/(size(traj_a,1)-num);
end
