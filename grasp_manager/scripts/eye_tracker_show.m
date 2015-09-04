% File: eye_tracker_show.m
% Description: During manual object alignment, it might be helpful to get a
%   second view of the scene. This program will open the inputted avi file
%   for the test being aligned and show specific time slices.

[filename, pathname] = uigetfile('*.wav','Select the audio of trial' ...
    ,'/media');
%grasp_data_dir = '/media/eva/FA648F24648EE2AD/grasp_data/';
%good_or_bad = input('good or bad?: ');
%obj_num = str2int32(input('obj num: '));
%sub_num = str2int32(input('sub num: '));

%audio_path = strcat(grasp_data_dir, good_or_bad, '/', 'obj', int2str(obj_num), '_sub', int2str(sub_num), '/smi_robot/S0', int2str(sub_num), '-3-recording.avi')

if (filename ~= 0)
    
    %values of 100 ms windows, with spike threshold of one have worked well
    %in the algorithm. 
    firstVidBeep = detectBeep(strcat(pathname,filename),5000,1/10,1);

    %calculate time in minutes, seconds, milliseconds
    remainder = mod(firstVidBeep,60);
    minutes = idivide(firstVidBeep, int32(60));

    strcat('First Video beep found at',{' '},int2str(minutes),' minutes, and' ...
           ,{' '},num2str(remainder),' seconds') 
end

%find annotation file  
[filename, pathname] = uigetfile('*.csv','Select the annotation file' ...
    ,pathname);

if (filename ~= 0)
    %uses xls read, these can't be read with csvread for some reason
    csv_matrix = read_mixed_csv(strcat(pathname,filename),',');
    
    % Find the stamp of the beep relative to the bag files
    %mocap_idx = -1;

    %for i = 1:size(columns,1)
    %    if strncmpi('Motion Capture', columns, 10) == 1
    %           mocap_idx = i;
    %    end
    %end
    mocap_idx = strmatch('Motion Capture Start', csv_matrix(:,3))
  
    vid_start_in_bagtime = (str2double(csv_matrix(mocap_idx,2)))/(10^9) - firstVidBeep;
    
    % Print out messages for reference
    fprintf('CSV messages:');
    %for i = 0:length(msgs)
    %    fprintf('%f %s\n', stamps(i), msgs(i));
    %end
    Frames_with_timestamp = csv_matrix(:,2:3)

end

% Open the eye tracking video
[avi_filename, avi_pathname] = uigetfile('*.mp4','Select the eye tracking video file' ...
    ,pathname);
eye_tracking_movie = VideoReader(strcat(avi_pathname, avi_filename));

while 1
    stamp = input('Input the timestamp you would like to see: ');
    eye_track_time = stamp/(10^9) - vid_start_in_bagtime
    
    % Get a frame at that time
    eye_tracking_movie.CurrentTime = eye_track_time;
    frame = eye_tracking_movie.readFrame();
    imshow(frame)
end
