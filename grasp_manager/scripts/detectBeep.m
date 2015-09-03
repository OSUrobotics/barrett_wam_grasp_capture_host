function [ retTime ] = detectBeep( filename, freq,fftWindowSize, spikeThreshold )
% function used to detect a beep of a given frequency in an audio
% signal. Assumes there is one beep in the audio file, and looks for 
% a rate of change in frequency power (amount of frequency present)
% that is greater than that rate of change. Returns the time for which that
% rate of change is met. Also graphs the response of that frequency
% over time in case the file contains multiple beeps, or to test 
% the threshold used.
%Input:
%   filename: name of the audio file to use (must be in same folder)
%   freq: a frequency in hertz to detect
%   fftWindowSize: in seconds. EX: 1 for one second intervals, 1/10 for
%       100ms, etc.
%   spikeThreshold: assuming there is one beep 
%Output:
%   t: time in seconds for a which a beep is detected

    %the SMI Eye Tracker mic encodes at 16 bit, so the 
    %data ranges from [-1.0,1.0) and fills y array
    %Fs is number of samples per second
    [y, Fs] = audioread(filename);

    %Number of samples
    %warning N uses the largest dimension
    N = length(y);
    
    windowsize = Fs * fftWindowSize;
    
    
    %spectoram performs FFT on y given a window size, # of overlap samples
    %frequency to use (I think this is unused) and samplerate. First index
    %of y used as we only are recording mono audio with SMI mic, the audio
    %in .avi is just copied to two channels.
    %Outputs:
    %   s: the spectrum itself, rows are bins of frequency
    %      cols are bins of time (based on window size).
    %      Note: a smaller window size makes ranges of freq bins less
    %      precise, but make time bins more precise. Note values of power
    %      are complex, and therefore should be viewed using abs() to get
    %      the magnitude of the complex number.
    %   f: An array of frequencies in Hz covered by spectrum. Has equal
    %       number of rows as s, and the indicies are aligned. Therefore
    %       finding the closest match to the given frequency in f will give
    %       us the index for which row of s we will look at. The value of f
    %       is for the small frequency in bin range.
    %   t: An array of time in seconds covered by spectrum. Has equal # of
    %       cols to cols in s. given an index in this you have the time
    %       that a specific bin in spectrum covers. The time value itself
    %       is the midpoint of the bin. Frequency response will be graphed
    %       over this array.
    [s,f,t] = spectrogram(y(:,1),windowsize,0,freq,Fs);
    
    %find closes value in f too 5k
    tmp = abs(f-freq);
    [idx idx] = min(tmp); %index of closest value
    
    freqResponse = abs(s(idx,:));
    
    figure('name','FFT Sanity Check, time should match the prominent peak!');
    
    plot(t,freqResponse,'--.');
    
    title(strcat('Response in',{' '},int2str(freq),' Hz over time'));
    xlabel('Time in seconds');
    ylabel('Amplitude');
    
    
    
    %might not use 
%     [pks,locs] = findpeaks(freqResponse,'MinPeakHeight',5);
%     
%     retTime = t(locs(1)-2);
    
    
    %time to get fancy, we'll calculate a rate of change array
    %t_n+1 - t_n, and then look for any change over the supplied threshold
    
    %our y_n+1
    freqResponse2 = freqResponse(2:end);
    freqResponse2(end+1) = 0;
    
    %rate of change
    slopes = freqResponse2 - freqResponse;
    %chose not to scale by time unit, as that means we should scale the
    %thershold param by time unit, which is redundant
    
    %loop until first slope that exceeds threshold
    for slope = slopes
       %scale the threshold by window size, as bigger windows
       %will have higher rates of change, smaller windows small changes
       if slope > spikeThreshold
           %return the seconds of which slope index plus one
           %as the jump is produced by the y_n+1 val, not the not y_n!
           retTime = t( find(slopes==slope) + 1);
           break;
       end
    end
    
    
    %there is a maximum of 50 ms worth of error when using 100 ms windows
    %, if we need more precise
    %results we could run an analysis on the 100ms window of data we have
    %selected. This requires a different parameter
end

