% File: eye_tracker_show.m
% Description: During manual object alignment, it might be helpful to get a
%   second view of the scene. This program will open the inputted avi file
%   for the test being aligned and show specific time slices.

[filename, pathname] = uigetfile({'*.wav'},'Select the audio of trial' ...
    ,'/media');
%grasp_data_dir = '/media/eva/FA648F24648EE2AD/grasp_data/';
%good_or_bad = input('good or bad?: ');
%obj_num = str2int32(input('obj num: '));
%sub_num = str2int32(input('sub num: '));

%audio_path = strcat(grasp_data_dir, good_or_bad, '/', 'obj', int2str(obj_num), '_sub', int2str(sub_num), '/smi_robot/S0', int2str(sub_num), '-3-recording.avi')

if (filename ~= 0)
    
    %values of 100 ms windows, with spike threshold of one have worked well
    %in the algorithm. 
    firstVidBeep = detectBeep(strcat(filename, pathname),5000,1/10,1);

    %calculate time in minutes, seconds, milliseconds
    remainder = mod(firstVidBeep,60);
    minutes = idivide(firstVidBeep, int32(60));

    strcat('First Video beep found at',{' '},int2str(minutes),' minutes, and' ...
           ,{' '},num2str(remainder),' seconds') 
end

%find annotation file  
[filename, pathname] = uigetfile('*.xls','Select the annotation file' ...
    ,pathname);

if (filename ~= 0)
    %uses xls read, these can't be read with csvread for some reason
    [stamps, msgs] = xlsread(strcat(pathname,filename));
    
    % Find the stamp of the beep relative to the bag files
    mocap_idx = -1;
    for i = 0:length(msgs)
        if strncmpi('Motion Capture', msgs, 10) == 1
               mocap_idx = i;
        end
    end
    
    vid_start_in_bagtime = stamps(mocap_idx) - firstVidBeep;
    
    % Print out messages for reference
    fprintf('CSV messages:');
    for i = 0:length(msgs)
        fprintf('%f %s\n', stamps(i), msgs(i));
    end
end

% Open the eye tracking video
[avi_filename, avi_pathname] = uigetfile('*.avi','Select the eye tracking video file' ...
    ,pathname);
eye_tracking_movie = VideoReader(strcat(avi_pathname, avi_filename));

while 1
    stamp = input('Input the timestamp you would like to see: ');
    eye_track_time = str2double(stamp) - vid_start_in_bagtime
    
    % Get a frame at that time
    eye_tracking_movie.CurrentTime = eye_track_time;
    frame = eye_tracking_movie.readframe()
    imshow(frame)
end