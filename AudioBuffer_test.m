% <purpose of this file>
% 
% Author :  J.-A. Adrian (JA) <jens-alrik.adrian AT jade-hs.de>
% Date   :  20-Mar-2015 13:04:55
% Updated:  <>

clear;
close all;

szFilename = 'Audio/speech_2chan.wav';
[signal, sampleRate] = audioread(szFilename);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
blocklenSec  = 30e-3;
overlapRatio = 0.88;
winfun = @(x) sqrt(hann(x, 'periodic'));

idxChannels = 1;
% idxChannels = [1 2];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% either call the class constructor with the signal vector and sample rate
% or with the filename
obj = AudioBuffer(signal, sampleRate);
% obj = AudioBuffer(szFilename);


obj.BlockLengthSec = blocklenSec;
obj.OverlapRatio   = overlapRatio;

obj.IdxChannels = idxChannels;

% either pass a function handle or a string of a function as the window
% function
obj.WindowFunction = winfun;
% obj.WindowFunction = 'hamming';


% do 'block processing'
blocklenSec = obj.BlockSize;
numBlocks   = obj.NumBlocks;
blockSignal = zeros(blocklenSec, length(idxChannels), numBlocks);
for aaBlock = 1:numBlocks,
    tic;
    blockSignal(:,:,aaBlock) = getBlock(obj);
    toc;
    
    figure(10);
    plot(blockSignal(:,:,aaBlock));
    axis([1 size(blockSignal,1) -1 1]);
    drawnow;
end

% Test if pulling another block really returns a warning
testdummy = obj.getBlock();

% Reconstruct the original signal with weighted-overlap-add (WOLA)
reconstructedSignal = zeros(size(signal,1),length(idxChannels));
for aaChan = 1:length(idxChannels),
    reconstructedSignal(:,aaChan) = WOLA(obj, squeeze(blockSignal(:,aaChan,:)));
end
    
figure(11);
plot([signal(:,idxChannels) reconstructedSignal]);






%-------------------- Licence ---------------------------------------------
% Copyright (c) 2015, J.-A. Adrian
% Institute for Hearing Technology and Audiology
% Jade University of Applied Sciences
% All rights reserved.
% 
% Redistribution and use in source and binary forms, with or without
% modification, are permitted provided that the following conditions are
% met:
%     * Redistributions of source code must retain the above copyright
%       notice, this list of conditions and the following disclaimer.
%     * Redistributions in binary form must reproduce the above copyright
%       notice, this list of conditions and the following disclaimer in the
%       documentation and/or other materials provided with the
%       distribution.
%     * Neither the name of the <organization> nor the
%       names of its contributors may be used to endorse or promote
%       products derived from this software without specific prior written
%       permission.
% 
% THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
% IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
% THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
% PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE
% FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
% CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
% SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
% INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
% CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
% ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
% THE POSSIBILITY OF SUCH DAMAGE.

% End of file: AudioBufferClass_test.m
