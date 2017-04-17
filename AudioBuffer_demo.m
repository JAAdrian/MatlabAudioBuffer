% Script to demonstrate the MATLAB AudioBuffer() class
%
% Author :  J.-A. Adrian (JA) <jensalrik.adrian AT gmail.com>
% Date   :  20-Mar-2015 13:04:55
%

clear;
close all;

load laughter;

sampleRate = Fs;
signal = y(1:round(2*sampleRate), 1);

clear y Fs;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Desired Parameters

blocklenSec  = 30e-3;
overlapRatio = 0.5;
winfun = @(x) sqrt(hann(x, 'periodic'));

idxChannels = 1;
% idxChannels = [1 2];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Instantiate Object
% either call the class constructor with the signal vector and sample rate
% or with a filename while setting the sample rate empty
obj = AudioBuffer(...
    signal, sampleRate, ...
    'BlockLengthSec', blocklenSec, ...
    'OverlapRatio', overlapRatio, ...
    'IdxChannels', idxChannels, ...
    'WindowFunction', winfun ...
    );
% obj = AudioBuffer( ...
%     filename, [], ...
%     'BlockLengthSec', blocklenSec, ...
%     'OverlapRatio', overlapRatio, ...
%     'IdxChannels', idxChannels, ...
%     'WindowFunction', winfun ...
%     );


%% Do 'Block Processing'
blocklenSec = obj.BlockSize;
numBlocks   = obj.NumBlocks;
blockSignal = zeros(blocklenSec, length(idxChannels), numBlocks);
for iBlock = 1:numBlocks
    tic;
    blockSignal(:,:,iBlock) = obj.step();
    toc;
    
    figure(10);
    plot(blockSignal(:,:,iBlock));
    grid on;
    axis([1 size(blockSignal,1) -1 1]);
    drawnow;
end

% Test if pulling another block really returns a warning
testdummy = obj.step();

% Reconstruct the original signal with weighted-overlap-add (WOLA)
reconstructedSignal = zeros(size(signal,1), length(idxChannels));
for iChan = 1:length(idxChannels)
    reconstructedSignal(:, iChan) = WOLA(obj, squeeze(blockSignal(:, iChan, :)));
end
    
figure(11);
plot([signal(:,idxChannels) reconstructedSignal]);
grid on;




% End of file: AudioBufferClass_test.m
