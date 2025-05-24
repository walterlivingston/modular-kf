function [time,dt,N] = setupTimeParams(ti,tf,Fs)

    arguments
        ti (1,1) double = 0;    % Initial Time [s]
        tf (1,1) double = 1;    % Final Time [s]
        Fs (1,1) double = 10;   % Sample Frequency [Hz]
    end

    dt = 1/Fs;                  % Time Step [s]
    time = ti:dt:tf;            % Time Vector [s]
    N = length(time);           % Length of Time Vector
end