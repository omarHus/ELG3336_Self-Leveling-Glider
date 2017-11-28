%%%%% Auto Tune PID Controller %%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear all, clc;

% connect to raspi
mypi = raspi();
% Set incoming file path
inPath = '/home/pi/Desktop/Glider Project/Flight Logs/flightlog-';
% File name
currentDate = date;
currentTime = '-'; %add time of file creation
fileName = currentDate;
fullName = strcat(inPath,currentDate,currentTime,'.log')
%%% Import CSV file (saves to current Directory) %%%
getFile(mypi, fullname);
fileIn = fopen(fullname,'r');  %might have to change fullname, to current directory

% read in all data
input = textscan(fileIn, '%f %f %f %f %f %f', 'Delimiter',',');
% parse data into pitch and roll components
rollIn = input(:,3);
pitchIn = input(:,4);

fclose(fileIn);
celldisp(rollIn);
celldisp(pitchIn);
fprintf('all done!');