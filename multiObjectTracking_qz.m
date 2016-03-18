function multiObjectTracking()
% Create system objects used for reading video, detecting moving objects,
% and displaying the results.//创建用于读取视频的系统对象，检测移动对象，并显示结果。
VideoName=input('input the video name:','s');
obj = setupSystemObjects(VideoName);

tracks = initializeTracks(); % Crea te a n empty array of tracks.//创建一个空数组。

nextId = 1; % ID of the next track//下一个轨道的标识
i_fram=0;
wid = 40;
hei = 60;
 load('GT_S01.mat','pos_ind');
 %显示背景
  set(gcf,'color','white');
    A=imread('picture1.jpg');
    B=imshow(A);
    hold on;
% Detect moving ob jects, and track them across video frames.//检测移动的物体，并跟踪他们的视频帧。
while ~isDone(obj.reader)  
        i_fram=i_fram+1;
    frame = readFrame();
    if i_fram~=1 && i_fram~=31 && i_fram~=661 && mod(i_fram-1,30)==0    
      centroids = pos_ind(i_fram==pos_ind(:,1),3:4);
      trajectory_show(centroids);
      bboxes = zeros(size(centroids,1),4);
      bboxes(:,1) = centroids(:,1)-wid/2;
      bboxes(:,2) = centroids(:,2)-hei/2;
      bboxes(:,3) = wid;
      bboxes(:,4) = hei;
    %   [centroids, bboxes, mask] = detectObjects(frame);
    predictNewLocationsOfTracks();
    [assignments, unassignedTracks, unassignedDetections] = ...
        detectionToTrackAssignment();

    updateAssignedTracks();
    updateUnassignedTracks();
    deleteLostTracks();
    createNewTracks();
    displayTrackingResults();
    savemat(VideoName);
    save_target(VideoName,i_fram); 
    end
end
labels=clear_short_traj(VideoName);
inte_traj_cluster(VideoName,labels);
function obj = setupSystemObjects(VideoName)
        % Initialize Video I/O//初始化视频输入/输出
        % Create objects for reading a video from a file, drawing the tracked//创建用于从文件读取视频的对象，绘制跟踪
        % objects in each frame, and playing the video.//在每一帧中的对象，并播放视频

        % Create a video file reader.//创建一个视频文件阅读器。
        obj.reader = vision.VideoFileReader(VideoName);%atrium

        % Create two video players, one to display the video,//创建2个视频播放器，一个显示视频，
        % and one to display the foreground mask.//和一个显示前景掩码。
        obj.videoPlayer = vision.VideoPlayer('Position', [20, 100, 700, 400]);
        obj.maskPlayer = vision.VideoPlayer('Position', [740, 400, 700, 400]);

        % Create system objects for foreground detection and blob
        % analysis //创建系统对象的前景检测和斑点分析

        % The foreground detector is used to segment moving objects from
        % the background. It outputs a binary mask, where the pixel value
        % of 1 corresponds to the foreground and the value of 0 corresponds
        % to the background.//前景检测用于从背景中获得段移动的对象。
        %//它输出的二进制掩码，其中的像素值1对应前景,0对应背景。

%         obj.detector = vision.ForegroundDetector('NumGaussians', 3, ...
%             'NumTrainingFrames', 40, 'MinimumBackgroundRatio', 0.7);

        % Connected groups of foreground pixels are likely to correspond to moving
        % objects.  The blob analysis system object is used to find such groups
        % (called 'blobs' or 'connected components'), and compute their
        % characteristics, such as area, centroid, and the bounding box.
%//前景像素的连接组有可能对应移动对象。Blob分析系统的对象是用来寻找这样的群体
%//（被称为“水滴”或“连接部件），并计算它们的
%//特征，如面积，质心，和包围盒。
%         obj.blobAnalyser = vision.BlobAnalysis('BoundingBoxOutputPort', true, ...
%             'AreaOutputPort', true, 'CentroidOutputPort', true, ...
%             'MinimumBlobArea', 400);
end
 function tracks = initializeTracks()
        % create an empty array of tracks//创建一个空数组
        %卡尔曼滤波是以最小均方误差为估计的最佳准则，
        %来寻求一套递推估计的算法，其基本思想是：采用信号与噪声的状态空间模型，
        %利用前一时刻的估计值和现时刻的观测值来更新对状态变量的估计，
        %求出现时刻的估计值。它适合于实时处理和计算机运算。
        tracks = struct(...
            'id', {}, ...
            'bbox', {}, ...
            'kalmanFilter', {}, ...
            'age', {}, ...
            'totalVisibleCount', {}, ...
            'consecutiveInvisibleCount', {});
 end
 function frame = readFrame()
        frame = obj.reader.step();
%          frame = imresize(frame,0.3);
 end
 
function predictNewLocationsOfTracks()
        for i = 1:length(tracks)
            bbox = tracks(i).bbox;

            % Predict the current location of the track.//预测当前位置的轨道。
            predictedCentroid = predict(tracks(i).kalmanFilter);

            % Shift the bounding box so that its center is at
            % the predicted location//移动的包围盒，使其中心是在预测位置
            predictedCentroid = int32(predictedCentroid) - int32(bbox(3:4)) / 2;
            tracks(i).bbox = [predictedCentroid, bbox(3:4)];
        end
end
 function [assignments, unassignedTracks, unassignedDetections] = ...
            detectionToTrackAssignment()

        nTracks = length(tracks);
        nDetections = size(centroids, 1);

        % Compute the cost of assigning each detection to each track.//计算每个跟踪的分配的成本。
        cost = zeros(nTracks, nDetections);
        for i = 1:nTracks
            cost(i, :) = distance(tracks(i).kalmanFilter, centroids);
        end

        % Solve the assignment problem.//解决指派问题。
       
        costOfNonAssignment = 20;
        [assignments, unassignedTracks, unassignedDetections] = ...
            assignDetectionsToTracks(cost, costOfNonAssignment);
 end
 function updateAssignedTracks()                              
              
       
        numAssignedTracks = size(assignments, 1);
        %.i=assignments(1,2);
        %for i=1:numAssignedTracks
         %   assignments(i,1)= assignments(i,2);
        %end
        for i = 1:numAssignedTracks
            trackIdx = assignments(i, 1);
            detectionIdx = assignments(i, 2);
            centroid = centroids(detectionIdx, :);
            bbox = bboxes(detectionIdx, :);

            % Correct the estimate of the object's location
            % using the new detection.//使用新的检测方法对对象的位置进行正确的估计。
            correct(tracks(trackIdx).kalmanFilter, centroid);

            % Replace predicted bounding box with detected
            % bounding box.//用检测包围盒替换预测的包围盒。
            tracks(trackIdx).bbox = bbox;

            % Update track's age.//更新轨迹的年龄。
            tracks(trackIdx).age = tracks(trackIdx).age + 1;

            % Update visibility.//更新能见度。
            tracks(trackIdx).totalVisibleCount = ...
                tracks(trackIdx).totalVisibleCount + 1;
            tracks(trackIdx).consecutiveInvisibleCount = 0;
        end
        
 end
   function updateUnassignedTracks()
        for i = 1:length(unassignedTracks)
            ind = unassignedTracks(i);
            tracks(ind).age = tracks(ind).age + 1;
            tracks(ind).consecutiveInvisibleCount = ...
                tracks(ind).consecutiveInvisibleCount + 1;
        end
   end
    function deleteLostTracks()
        if isempty(tracks)
            return;
        end

        invisibleForTooLong = 10;
        ageThreshold = 8;

        % Compute the fraction of the track's age for which it was
        % visible.//计算轨道的年龄段，它是可见的。
        ages = [tracks(:).age];
        totalVisibleCounts = [tracks(:).totalVisibleCount];
        visibility = totalVisibleCounts ./ ages;

        % Find the indices of 'lost' tracks.
        lostInds = (ages < ageThreshold & visibility < 0.6) | ...
            [tracks(:).consecutiveInvisibleCount] >= invisibleForTooLong;

        % Delete lost tracks.//寻找丢失的足迹。
        tracks = tracks(~lostInds);
    end
    function createNewTracks()
        centroids = centroids(unassignedDetections, :);
        bboxes = bboxes(unassignedDetections, :);

        for i = 1:size(centroids, 1)

            centroid = centroids(i,:);
            bbox = bboxes(i, :);

            % Create a Kalman filter object.//创建一个卡尔曼滤波对象。
            kalmanFilter = configureKalmanFilter('ConstantVelocity', ...
                centroid, [200, 50], [100, 25], 100);

            % Create a new track.
            newTrack = struct(...
                'id', nextId, ...
                'bbox', bbox, ...
                'kalmanFilter', kalmanFilter, ...
                'age', 1, ...
                'totalVisibleCount', 1, ...
                'consecutiveInvisibleCount', 0);

            % Add it to the array of tracks.//创建一个新的轨道。
            tracks(end + 1) = newTrack;

            % Increment the next id.//增加下一个编号。 
            nextId = nextId + 1;
        end
    end
function displayTrackingResults()
        % Convert the frame and the mask to uint8 RGB.//框架和面具uint8 RGB。
        frame = im2uint8(frame);
%         mask = uint8(repmat(mask, [1, 1, 3])) .* 255;

        minVisibleCount = 0;
        if ~isempty(tracks)

            % Noisy detections tend to result in short-lived tracks.
            % Only display tracks that have been visible for more than
            % a minimum number of frames.//噪声检测往往导致轨道停止。只有显示的轨道，已经超过了最小数目的帧
            reliableTrackInds = ...
                [tracks(:).totalVisibleCount] > minVisibleCount;
            reliableTracks = tracks(reliableTrackInds);

            % Display the objects. If an object has not been detected
            % in this frame, display its predicted bounding box.//显示对象。如果未检测到对象,在这个框架中，显示其预测的边界框。
            if ~isempty(reliableTracks)
                % Get bounding boxes.//得到包围盒。
                bboxes = cat(1, reliableTracks.bbox);

                % Get ids.//获得ids
                ids = int32([reliableTracks(:).id]);

                % Create labels for objects indicating the ones for
                % which we display the predicted rather than the actual
                % location.为对象创建标签，表示其为我们展示的预测，而不是实际位置。
                labels = cellstr(int2str(ids'));
                predictedTrackInds = ...
                    [reliableTracks(:).consecutiveInvisibleCount] > 0;
                isPredicted = cell(size(labels));
                isPredicted(predictedTrackInds) = {' predicted'};
                labels = strcat(labels, isPredicted);

                % Draw the objects on the frame.//把物体画在框架上。
                frame = insertObjectAnnotation(frame, 'rectangle', ...
                    bboxes, labels);

                % Draw the objects on the mask.画上掩码的对象。
%                 mask = insertObjectAnnotation(mask, 'rectangle', ...
%                     bboxes, labels);
            end
        end

        % Display the mask and the frame.//显示掩码和框架。
       % obj.maskPlayer.step(mask);
        obj.videoPlayer.step(frame);
end
function savemat(VideoName)
    workdir = VideoName(1:find(VideoName=='.')-1);
    if  ~exist(workdir) 
    mkdir(workdir)  
    end
    workdir = strcat(workdir,'/');    
    savtra = sprintf('track%d.mat', i_fram);
     savtra = strcat(workdir,savtra);
     save (savtra, 'tracks');
     
    savfile = sprintf('tracks%d.jpg', i_fram);%task
    savfile = strcat(workdir,savfile);
    imwrite(frame,savfile);  
  end
end

%……显示轨迹
function trajectory_show(centroids)
    figure(1);
    for i=1:size(centroids,1)
    x=centroids(i,1);
    y=centroids(i,2);
    plot(x,y,'b:.','MarkerSize',20);%'MarkerFaceColor','r'
    set(gca,'ydir','reverse');
    hold on;
    axis([0 1000 0 700]);    
    end
end
%……保存目标txt格式，如1.txt，1代表第一个目标
function save_target(VideoName,i_fram)
 workdir = VideoName(1:find(VideoName=='.')-1);
 workdir = strcat(workdir,'\');    
 savtra = sprintf('track%d.mat', i_fram);
 savtra = strcat(workdir,savtra);
load(savtra);
 res_ids = [];
 res_ids = [res_ids tracks(:).id];
 uniqueId = unique(res_ids);
% workdir='G:/SG/';
 for i_uni = 1:length(uniqueId)
     finds = find(res_ids==uniqueId(i_uni));
     obj_fSum = length(finds);
    staticId = zeros(obj_fSum,5);
     n=1;
     for j_f=finds
        staticId(n,:) =[i_fram,tracks(j_f).bbox];
         n=n+1;
     end
     outputFileName = sprintf('result_%d.txt', uniqueId(i_uni)); %for Scurv.avi
      outputFileName = strcat(workdir,outputFileName);
    % outputFileName=strcat('result_',num2str(uniqueId(i_uni)),'.txt');
     dlmwrite(outputFileName,staticId,'-append');
 end
end

%%清除较短的轨迹
function labels=clear_short_traj(VideoName)
    ans=pwd;
    ans1=strcat(ans,'\');
    ans2=strcat(ans,'\',VideoName(1:find(VideoName=='.')-1));
    addpath(ans2);
    bns=strcat(ans2,'\*.txt');
    bns1=strcat(ans2,'\');
    file=dir(bns);
    labels=[]
    for  n=1:length(file)
        x=strcat('result_',num2str(n),'.txt');
        y=load(x);
        if size(y,1)<5
            labels(n)=0;
        else 
            labels(n)=1;
        end
    end
end
%将轨迹综合
function inte_traj_cluster(VideoName,labels)
     %%整理轨迹数据 保留x，y值
    ans=pwd;
    ans1=strcat(ans,'\');
    ans2=strcat(ans,'\');
    addpath(ans2);
    bns=strcat(ans2,VideoName(1:find(VideoName=='.')-1),'\');
    bns1=strcat(bns,'\*.txt');
    file=dir(bns1);
    %%建立矩阵
    maxline=0;
    for n=1:length(file)    
        cc=[bns,file(n).name];
        c=load(cc);
       if maxline<size(c)
           maxline=size(c,1);
       end
    end
    %%合并数据
    result=[];
    for n=1:length(file)   
        if labels(n)==1
        B_temp = zeros(maxline, 1);
        C_temp = zeros(maxline, 1);
        cc=[bns,file(n).name];
        [data1 B C  data4 data5] = textread(cc,'%f%f%f%f%f','delimiter', ',')
        %[B,C]=textread('result_1.txt','%*%f,%f%*[^\n]','delimiter', ',')
        fid=fopen([ans2,file(n).name]);
       % B=textscan(fid,'%f,%f,%f,%f,%f');
        %C=[B{2} B{3}];
    %    B_temp(1:length(B)) = B{2};
     %   C_temp(1:length(C)) = B{3};
        B_temp(1:length(B)) = B;
        C_temp(1:length(C)) = C;    
        result=[result B_temp C_temp]    
        end
    end
    %%求解关系矩阵
    R_matrix=ones(size(result,2)/2);
    for i_f=1:size(result,2)/2
        traj_a=[result(:,2*i_f-1) result(:,2*i_f)];
        for j_f=1:size(result,2)/2
            if i_f==j_f
                 R_matrix(i_f,j_f)=0;
            else
            traj_b=[result(:,2*j_f-1) result(:,2*j_f)];
            R_matrix(i_f,j_f)=traj_compare(traj_a,traj_b);
            end
        end
    end
   
    K=2;
    label= kmeans(R_matrix,K)
     for i=1:size(label)
         trajectory=[result(:,2*i-1) result(:,2*i)]
          show_traj(trajectory,label(i));
    end
end
%%欧式距离
function distance=E_distance(a,b,c,d)

   distance =sqrt((a-c)*(a-c)+(b-d)*(b-d));
end
%显示轨迹
function show_traj(trajectory,label)
        figure(2);
     for i=1:size(trajectory,1) 
         if label==1,
          plot(trajectory(i,1),trajectory(i,2),'r*') % 显示第一类
           set(gca,'ydir','reverse');
         %plot(x(i,2),'r*') % 显示第一类
         hold on 
       else 
           if label==2, 
             plot(trajectory(i,1),trajectory(i,2),'b*') %显示第二类 
              set(gca,'ydir','reverse');
             %  plot(x(i,2),'b*') % 显示第一类
               hold on 
           end
       end
    end

end
%%轨迹相似性测量
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
