function show_traj(trajectory,labels)
     for i=1:size(trajectory,1) 
         if labels==1,
          plot(trajectory(i,1),trajectory(i,2),'r*') % ��ʾ��һ��
           set(gca,'ydir','reverse');
         %plot(x(i,2),'r*') % ��ʾ��һ��
         hold on 
       else 
           if labels==2, 
             plot(trajectory(i,1),trajectory(i,2),'b*') %��ʾ�ڶ��� 
              set(gca,'ydir','reverse');
             %  plot(x(i,2),'b*') % ��ʾ��һ��
               hold on 
           end
       end
    end

end