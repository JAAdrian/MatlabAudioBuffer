% <purpose of this file>
% 
% Author :  J.-A. Adrian (JA) <jens-alrik.adrian AT jade-hs.de>
% Date   :  20-Mar-2015 13:04:55
% Updated:  <>

clear;
close all;

szFilename = 'Audio/speech_2chan.wav';
[vSignal,fs] = audioread(szFilename);

% fs = 16e3;
% vSignal = randn(round(9e-3*fs),1);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
blocklen = 30e-3;
overlap  = 0.5;
hWin     = @(x) sqrt(hann(x,'periodic'));

vIdxChannels = 1;
% vIdxChannels = [1 2];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


cAudioBuffer = AudioBuffer(vSignal,fs);
% cAudioBuffer = AudioBuffer(szFilename);


cAudioBuffer.BlocklenSec  = blocklen;
cAudioBuffer.OverlapRatio = overlap;

cAudioBuffer.IdxChannels = vIdxChannels;

cAudioBuffer.WindowFunction = hWin;
% cAudioBuffer.WindowFunction = 'hann';


blocklen     = cAudioBuffer.Blocklen;
numBlocks    = cAudioBuffer.NumBlocks;
mBlockSignal = zeros(blocklen,length(vIdxChannels),numBlocks);
for aaBlock = 1:numBlocks,
    tic;
    mBlockSignal(:,:,aaBlock) = cAudioBuffer.getBlock();
    toc;
    
    figure(10);
    plot(mBlockSignal(:,:,aaBlock));
    axis([1 size(mBlockSignal,1) -1 1]);
    drawnow;
end
% dummy = cAudioBuffer.getBlock();

vReconstSignal = zeros(size(vSignal,1),length(vIdxChannels));
for aaChan = 1:length(vIdxChannels),
    vReconstSignal(:,aaChan) = cAudioBuffer.unbuffer(squeeze(mBlockSignal(:,aaChan,:)));
end
    
figure(11);
plot([vSignal(:,vIdxChannels) vReconstSignal]);






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
