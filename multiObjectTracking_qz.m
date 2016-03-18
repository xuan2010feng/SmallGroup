function multiObjectTracking()
% Create system objects used for reading video, detecting moving objects,
% and displaying the results.//�������ڶ�ȡ��Ƶ��ϵͳ���󣬼���ƶ����󣬲���ʾ�����
VideoName=input('input the video name:','s');
obj = setupSystemObjects(VideoName);

tracks = initializeTracks(); % Crea te a n empty array of tracks.//����һ�������顣

nextId = 1; % ID of the next track//��һ������ı�ʶ
i_fram=0;
wid = 40;
hei = 60;
 load('GT_S01.mat','pos_ind');
 %��ʾ����
  set(gcf,'color','white');
    A=imread('picture1.jpg');
    B=imshow(A);
    hold on;
% Detect moving ob jects, and track them across video frames.//����ƶ������壬���������ǵ���Ƶ֡��
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
        % Initialize Video I/O//��ʼ����Ƶ����/���
        % Create objects for reading a video from a file, drawing the tracked//�������ڴ��ļ���ȡ��Ƶ�Ķ��󣬻��Ƹ���
        % objects in each frame, and playing the video.//��ÿһ֡�еĶ��󣬲�������Ƶ

        % Create a video file reader.//����һ����Ƶ�ļ��Ķ�����
        obj.reader = vision.VideoFileReader(VideoName);%atrium

        % Create two video players, one to display the video,//����2����Ƶ��������һ����ʾ��Ƶ��
        % and one to display the foreground mask.//��һ����ʾǰ�����롣
        obj.videoPlayer = vision.VideoPlayer('Position', [20, 100, 700, 400]);
        obj.maskPlayer = vision.VideoPlayer('Position', [740, 400, 700, 400]);

        % Create system objects for foreground detection and blob
        % analysis //����ϵͳ�����ǰ�����Ͱߵ����

        % The foreground detector is used to segment moving objects from
        % the background. It outputs a binary mask, where the pixel value
        % of 1 corresponds to the foreground and the value of 0 corresponds
        % to the background.//ǰ��������ڴӱ����л�ö��ƶ��Ķ���
        %//������Ķ��������룬���е�����ֵ1��Ӧǰ��,0��Ӧ������

%         obj.detector = vision.ForegroundDetector('NumGaussians', 3, ...
%             'NumTrainingFrames', 40, 'MinimumBackgroundRatio', 0.7);

        % Connected groups of foreground pixels are likely to correspond to moving
        % objects.  The blob analysis system object is used to find such groups
        % (called 'blobs' or 'connected components'), and compute their
        % characteristics, such as area, centroid, and the bounding box.
%//ǰ�����ص��������п��ܶ�Ӧ�ƶ�����Blob����ϵͳ�Ķ���������Ѱ��������Ⱥ��
%//������Ϊ��ˮ�Ρ������Ӳ����������������ǵ�
%//����������������ģ��Ͱ�Χ�С�
%         obj.blobAnalyser = vision.BlobAnalysis('BoundingBoxOutputPort', true, ...
%             'AreaOutputPort', true, 'CentroidOutputPort', true, ...
%             'MinimumBlobArea', 400);
end
 function tracks = initializeTracks()
        % create an empty array of tracks//����һ��������
        %�������˲�������С�������Ϊ���Ƶ����׼��
        %��Ѱ��һ�׵��ƹ��Ƶ��㷨�������˼���ǣ������ź���������״̬�ռ�ģ�ͣ�
        %����ǰһʱ�̵Ĺ���ֵ����ʱ�̵Ĺ۲�ֵ�����¶�״̬�����Ĺ��ƣ�
        %�����ʱ�̵Ĺ���ֵ�����ʺ���ʵʱ����ͼ�������㡣
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

            % Predict the current location of the track.//Ԥ�⵱ǰλ�õĹ����
            predictedCentroid = predict(tracks(i).kalmanFilter);

            % Shift the bounding box so that its center is at
            % the predicted location//�ƶ��İ�Χ�У�ʹ����������Ԥ��λ��
            predictedCentroid = int32(predictedCentroid) - int32(bbox(3:4)) / 2;
            tracks(i).bbox = [predictedCentroid, bbox(3:4)];
        end
end
 function [assignments, unassignedTracks, unassignedDetections] = ...
            detectionToTrackAssignment()

        nTracks = length(tracks);
        nDetections = size(centroids, 1);

        % Compute the cost of assigning each detection to each track.//����ÿ�����ٵķ���ĳɱ���
        cost = zeros(nTracks, nDetections);
        for i = 1:nTracks
            cost(i, :) = distance(tracks(i).kalmanFilter, centroids);
        end

        % Solve the assignment problem.//���ָ�����⡣
       
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
            % using the new detection.//ʹ���µļ�ⷽ���Զ����λ�ý�����ȷ�Ĺ��ơ�
            correct(tracks(trackIdx).kalmanFilter, centroid);

            % Replace predicted bounding box with detected
            % bounding box.//�ü���Χ���滻Ԥ��İ�Χ�С�
            tracks(trackIdx).bbox = bbox;

            % Update track's age.//���¹켣�����䡣
            tracks(trackIdx).age = tracks(trackIdx).age + 1;

            % Update visibility.//�����ܼ��ȡ�
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
        % visible.//������������Σ����ǿɼ��ġ�
        ages = [tracks(:).age];
        totalVisibleCounts = [tracks(:).totalVisibleCount];
        visibility = totalVisibleCounts ./ ages;

        % Find the indices of 'lost' tracks.
        lostInds = (ages < ageThreshold & visibility < 0.6) | ...
            [tracks(:).consecutiveInvisibleCount] >= invisibleForTooLong;

        % Delete lost tracks.//Ѱ�Ҷ�ʧ���㼣��
        tracks = tracks(~lostInds);
    end
    function createNewTracks()
        centroids = centroids(unassignedDetections, :);
        bboxes = bboxes(unassignedDetections, :);

        for i = 1:size(centroids, 1)

            centroid = centroids(i,:);
            bbox = bboxes(i, :);

            % Create a Kalman filter object.//����һ���������˲�����
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

            % Add it to the array of tracks.//����һ���µĹ����
            tracks(end + 1) = newTrack;

            % Increment the next id.//������һ����š� 
            nextId = nextId + 1;
        end
    end
function displayTrackingResults()
        % Convert the frame and the mask to uint8 RGB.//��ܺ����uint8 RGB��
        frame = im2uint8(frame);
%         mask = uint8(repmat(mask, [1, 1, 3])) .* 255;

        minVisibleCount = 0;
        if ~isempty(tracks)

            % Noisy detections tend to result in short-lived tracks.
            % Only display tracks that have been visible for more than
            % a minimum number of frames.//��������������¹��ֹͣ��ֻ����ʾ�Ĺ�����Ѿ���������С��Ŀ��֡
            reliableTrackInds = ...
                [tracks(:).totalVisibleCount] > minVisibleCount;
            reliableTracks = tracks(reliableTrackInds);

            % Display the objects. If an object has not been detected
            % in this frame, display its predicted bounding box.//��ʾ�������δ��⵽����,���������У���ʾ��Ԥ��ı߽��
            if ~isempty(reliableTracks)
                % Get bounding boxes.//�õ���Χ�С�
                bboxes = cat(1, reliableTracks.bbox);

                % Get ids.//���ids
                ids = int32([reliableTracks(:).id]);

                % Create labels for objects indicating the ones for
                % which we display the predicted rather than the actual
                % location.Ϊ���󴴽���ǩ����ʾ��Ϊ����չʾ��Ԥ�⣬������ʵ��λ�á�
                labels = cellstr(int2str(ids'));
                predictedTrackInds = ...
                    [reliableTracks(:).consecutiveInvisibleCount] > 0;
                isPredicted = cell(size(labels));
                isPredicted(predictedTrackInds) = {' predicted'};
                labels = strcat(labels, isPredicted);

                % Draw the objects on the frame.//�����廭�ڿ���ϡ�
                frame = insertObjectAnnotation(frame, 'rectangle', ...
                    bboxes, labels);

                % Draw the objects on the mask.��������Ķ���
%                 mask = insertObjectAnnotation(mask, 'rectangle', ...
%                     bboxes, labels);
            end
        end

        % Display the mask and the frame.//��ʾ����Ϳ�ܡ�
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

%������ʾ�켣
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
%��������Ŀ��txt��ʽ����1.txt��1�����һ��Ŀ��
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

%%����϶̵Ĺ켣
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
%���켣�ۺ�
function inte_traj_cluster(VideoName,labels)
     %%����켣���� ����x��yֵ
    ans=pwd;
    ans1=strcat(ans,'\');
    ans2=strcat(ans,'\');
    addpath(ans2);
    bns=strcat(ans2,VideoName(1:find(VideoName=='.')-1),'\');
    bns1=strcat(bns,'\*.txt');
    file=dir(bns1);
    %%��������
    maxline=0;
    for n=1:length(file)    
        cc=[bns,file(n).name];
        c=load(cc);
       if maxline<size(c)
           maxline=size(c,1);
       end
    end
    %%�ϲ�����
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
    %%����ϵ����
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
%%ŷʽ����
function distance=E_distance(a,b,c,d)

   distance =sqrt((a-c)*(a-c)+(b-d)*(b-d));
end
%��ʾ�켣
function show_traj(trajectory,label)
        figure(2);
     for i=1:size(trajectory,1) 
         if label==1,
          plot(trajectory(i,1),trajectory(i,2),'r*') % ��ʾ��һ��
           set(gca,'ydir','reverse');
         %plot(x(i,2),'r*') % ��ʾ��һ��
         hold on 
       else 
           if label==2, 
             plot(trajectory(i,1),trajectory(i,2),'b*') %��ʾ�ڶ��� 
              set(gca,'ydir','reverse');
             %  plot(x(i,2),'b*') % ��ʾ��һ��
               hold on 
           end
       end
    end

end
%%�켣�����Բ���
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
    %num����켣�е�nan��ֵ�����ų���
     num=numel(traj_a(isnan(traj_a)))/2;
     z= trajectorydistance/(size(traj_a,1)-num);
end
