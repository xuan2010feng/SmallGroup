function show_traj(trajectory,labels)
     for i=1:size(trajectory,1) 
         if labels==1,
          plot(trajectory(i,1),trajectory(i,2),'r*') % 显示第一类
           set(gca,'ydir','reverse');
         %plot(x(i,2),'r*') % 显示第一类
         hold on 
       else 
           if labels==2, 
             plot(trajectory(i,1),trajectory(i,2),'b*') %显示第二类 
              set(gca,'ydir','reverse');
             %  plot(x(i,2),'b*') % 显示第一类
               hold on 
           end
       end
    end

end