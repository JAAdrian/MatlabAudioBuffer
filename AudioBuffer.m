classdef AudioBuffer < handle
%AUDIOBUFFER Class providing methods for convenient audio block processing
% -------------------------------------------------------------------------
% AudioBuffer procuces an object providing methods to conveniently work
% with traditional audio block processing.
% 
% AudioBuffer Properties:
%    SignalDim - Signal Dimensions
%    numBlocks - Number of Signal Blocks
%     blocklen - Block Length in Samples
%           fs - Sampling Frequency
%  blocklenSec - Block Length in Seconds
% overlapRatio - Ratio of Overlapping Samples
%  szWindowFun - Name of the Window Function
%    vIdxChans - Chosen Signal Channels
%
% AudioBuffer Methods:
%    getBlock - get a current block of audio data
%    unbuffer - reconstruct the original signal via WOLA
% 
% 
% Example:
% =========================================================================
%   obj = AudioBuffer(vSignal,fs,30e-3,0.5,@hann);
% 
%   numBlocks = obj.numBlocks;
%   blocklen  = obj.blocklen;
% 
%   mSignalBlocks = zeros(blocklen,numBlocks);
%   for aaBlock = 1:numBlocks,
%       mSignalBlocks(:,aaBlock) = obj.getBlock;
%   end
% 
%   vReconstructedSig = obj.unbuffer(mSignalBlocks);
% 
%   norm(vSignal - vReconstructedSig)^2
% 
%   ans =
% 
%     0.0021
% =========================================================================
% 
% 
% Author :  J.-A. Adrian (JA) <jens-alrik.adrian AT jade-hs.de>
% Date   :  20-Mar-2015 13:04:55
% Updated:  <>
%


properties (Access = private)
    vSignal;

    overlap;
    szFilename;
    szRetrievalFun;
    stAudioInfo;
    lenSignal;
    
    hWindow = @rectwin;
    vWindow;
    
    frameshift;
    lenPaddedSignal;
    remSamples;
    
    vThisBlock;
    
    isFinished = false;
end
properties (SetAccess = private)

%SignalDim Signal Dimensions.
%   Signal Dimensions of the chosen signal vector/matrix or file, i.e.
%   typically [numSamples x numChans].
    SignalDim;
   
%numBlocks Number of Signal Blocks
%   Number of blocks based on the chosen block length and overlap. Per
%   default the last block will be padded with zeros if it lacks samples.
    numBlocks;
   
%blocklen Block Length in Samples.
%   Block length in samples based on the chosen length in seconds and the
%   sampling frequency.
    blocklen;
  
%fs Sampling Frequency.
%   Sampling rate which has been passed or read from the chosen audio file.
    fs = [];
    
%blocklenSec Block Length in Seconds.
%   Block length in seconds. The default is 30 ms as it is usual in speech
%   processing.
    blocklenSec  = 30e-3;
    
%overlapRatio Ratio of Overlapping Samples
%   Overlap given as ratio re. to the block length. The default is 0.5,
%   i.e. 50%.
    overlapRatio = 0.5;
       
%szWindowFun Name of the Window Function
%   The default window function is a rectangular window so all further data
%   management is applied outside the object. If a different window
%   function has been passed as a handle, its name is displayed as string.
    szWindowFun = 'rectwin';
end

properties
%vIdxChans Chosen Signal Channels
%   Scalar or vector defining which channel shall be processed from the
%   signal vector/matrix or file. The default is 1, i.e. the left channel
%   in stereo signals.
    vIdxChans = 1;
end





methods
    % constructor
    function self = AudioBuffer(Source,fs,blocklenSec,overlapRatio,hWinFun)
        % Constructor of the AudioBuffer Class.
        % Usage:
        %
        %	self = AudioBuffer(Source,fs,{blocklenSec},{overlapRatio},{hWinFun})
        % 
        % blocklenSec is optional (default: 30ms) and overlapRatio is
        % optional (default: 0.5)
        
        if nargin,
            if nargin > 1 && ~isempty(fs),
                self.fs = fs;
            end
            if nargin > 2 && ~isempty(blocklenSec),
                self.blocklenSec  = blocklenSec;
            end
            if nargin > 3 && ~isempty(overlapRatio),
                self.overlapRatio = overlapRatio;
            end
            if nargin > 4 && ~isempty(hWinFun),
                self.hWindow = hWinFun;
            end
            
            if isnumeric(Source),           % 'Source' is a data vector/matrix
                self.vSignal        = Source;
                self.szRetrievalFun = 'cutIntoBlocks';
                self.SignalDim      = size(self.vSignal);
            elseif ischar(Source),          % 'Source' is a path to a file
                self.szFilename     = Source;
                self.szRetrievalFun = 'readFromAudioFile';
                self.stAudioInfo    = audioinfo(Source);
                self.SignalDim      = [self.stAudioInfo.TotalSamples, self.stAudioInfo.NumChannels];
            else
                error(['Source is not recognized!', ...
                    'Pass a signal vector/matrix',...
                    'or a path to an audio file (e.g. C:/.../audio.wav)']);
            end
        
            if isempty(self.stAudioInfo),
                self.fs        = fs;
                self.lenSignal = size(self.vSignal,1);
            else
                self.fs        = self.stAudioInfo.SampleRate;
                self.lenSignal = self.stAudioInfo.TotalSamples;
            end
            
            
        end

        
        self.blocklen    = round(self.blocklenSec * self.fs);
        self.overlap     = round(self.blocklen * self.overlapRatio);
        self.frameshift  = self.blocklen - self.overlap;
        self.vWindow     = self.hWindow(self.blocklen);
        self.szWindowFun = func2str(self.hWindow);
        
        % have at least 1 data block. Can occur if the signal is shorter
        % than the block length and shorter than the overlap
        self.numBlocks = max(1,ceil((self.lenSignal - self.overlap) / self.frameshift));
        self.lenPaddedSignal = self.numBlocks * self.frameshift + self.overlap;
        
        self.remSamples = self.lenPaddedSignal - self.lenSignal;
        
        self.vThisBlock = 1:self.blocklen;
        
        self.isFinished = false;
    end
    
    function [vData] = getBlock(self)
        % Essential method to optain a current block of the audio signal
        
        if self.isFinished,
            warning('The last data block has been returned. No more data left!');
            vData = [];
            return;
        end
        
        
        % --------------------------------------------------------------- %
        if self.vThisBlock(end) < self.lenSignal,
            vData = self.(self.szRetrievalFun)(self.vThisBlock);
            vData = vData(:,self.vIdxChans);
        else
            vData = self.(self.szRetrievalFun)(self.vThisBlock(1):self.lenSignal);
            vData = [vData(:,self.vIdxChans); zeros(self.remSamples,length(self.vIdxChans))];
            
            self.isFinished = true;
        end
        vData = vData .* (self.vWindow * ones(1,length(self.vIdxChans)));
        % --------------------------------------------------------------- %
        
        self.vThisBlock = self.vThisBlock + self.frameshift;
    end
    
    function vSignalOut = unbuffer(self,mSignalBlocks)
        [blocklenIn,numBlocksIn] = size(mSignalBlocks);
        
        if numBlocksIn < self.numBlocks || blocklenIn < self.blocklen,
            error(['Use this function only with data retrievend from ',...
                'this class. Block length and/or number of data blocks does not fit!']);
        end
        
        mSignalBlocks = diag(sparse(self.vWindow)) * mSignalBlocks;
        
        vIdxBlock = 1:self.blocklen;
        
        vSignalOut = zeros(self.lenPaddedSignal,1);
        for aaFrame = 1:self.numBlocks,
            vSignalOut(vIdxBlock) = ...
                vSignalOut(vIdxBlock) + mSignalBlocks(:,aaFrame);
            
            vIdxBlock = vIdxBlock + self.frameshift;
        end
        
        vSignalOut = vSignalOut(1:self.lenSignal);
    end
end



methods (Access = private)
    function [vData] = cutIntoBlocks(self,vBlockIdx)
        vData = self.vSignal(vBlockIdx,:);
    end
    
    function [vData] = readFromAudioFile(self,vBlockIdx)
        vData = audioread(self.szFilename,[vBlockIdx(1), vBlockIdx(end)]);
    end
end
    
end





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

% End of file: AudioBufferClass.m
