classdef AudioBuffer < handle
%AUDIOBUFFER Class providing methods for convenient audio block processing
% -------------------------------------------------------------------------
% AudioBuffer procuces an object providing methods to conveniently work
% with traditional audio block processing. As input can be used a long
% signal vector or a filename pointing to an audio file.
% The essential method `getBlock` returns one signal block at a time
% according to the desired parameters block length and overlap.
% 
% 
% AudioBuffer Properties:
%    SignalDimensions - Signal Dimensions
%    numBlocks - Number of Signal Blocks
%     Blocklength - Block Length in Samples
%           fs - Sampling Frequency
%  BlocklengthSec - Block Length in Seconds
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
%   Blocklength  = obj.Blocklength;
% 
%   mSignalBlocks = zeros(Blocklength,numBlocks);
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
% Author:  J.-A. Adrian (JA) <jens-alrik.adrian AT jade-hs.de>
% Date  :  20-Mar-2015 13:04:55
% 

% Version: v0.1 initial release, 20-Mar-2015 (JA)
%          v1.0 added input checks, improve documentation, 16-Mar-2016 (JA)


properties (Access = private)
    Signal;
 
    Filename;
    RetrievalFun;
    
    ThisBlock;
    
    IsFinished = false;
end

properties (Access = private, Dependent)
    Overlap;
    Frameshift;
    LengthPaddedSignal;
    RemainingSamples;
end

properties (SetAccess = private)
%Fs Sampling Frequency.
%   Sampling rate which has been passed or read from the chosen audio file.
    Fs;
    
%SignalDimensions Signal Dimensions.
%   Signal Dimensions of the chosen signal vector/matrix or file, i.e.
%   typically [numSamples x numChans].
    SignalDimensions;
end

properties (SetAccess = private, Dependent)
%NumBlocks Number of Signal Blocks
%   Number of blocks based on the chosen block length and overlap. Per
%   default the last block will be padded with zeros if it lacks samples.
    NumBlocks;
    
%Blocklength Block Length in Samples.
%   Block length in samples based on the chosen length in seconds and the
%   sampling frequency.
    Blocklength;
end

properties
%BlocklengthSec Block Length in Seconds.
%   Block length in seconds. The default is 30 ms as it is usual in speech
%   processing.
    BlocklengthSec  = 30e-3;
    
%OverlapRatio Ratio of Overlapping Samples
%   Overlap given as ratio re. to the block length. The default is 0,
%   i.e. 0 * 100%, due to the default rectangular window.
    OverlapRatio = 0.5;
    
%WindowFunction Window Function Applied to the Signal Block
%   The default is '@rectwin' in conjunction with OverlapRatio = 0. Thus,
%   only the raw signal blocks are retrieved.
%   Pass the window function either as function handle, e.g. as
%   obj.WindowFunction = @(x) sqrt(hann(x, 'period'));
%   or as string corresponding to the function to be used, e.g. as
%   obj.WindowFunction = 'hann';
    WindowFunction = @rectwin;
    
%IdxChannels Chosen Signal Channels
%   Scalar or vector defining which channel shall be processed from the
%   signal vector/matrix or file. The default is 1, i.e. the left channel
%   in stereo signals.
    IdxChannels = 1;
end





methods
    % constructor
    function self = AudioBuffer(Source, fs)
        % Constructor of the AudioBuffer Class.
        % Usage:
        %
        %	self = AudioBuffer(Source,fs)
        % 
        % BlocklengthSec is optional (default: 30ms) and overlapRatio is
        % optional (default: 0.5)
        
        if nargin,            
            if isnumeric(Source),           % 'Source' is a data vector/matrix
                self.Signal        = Source;
                self.RetrievalFun = 'cutIntoBlocks';
                
                if nargin > 1 && ~isempty(fs),
                    self.Fs = fs;
                end
                
                self.SignalDimensions = size(self.Signal);
                
            elseif ischar(Source),          % 'Source' is a path to a file
                self.Filename     = Source;
                self.RetrievalFun = 'readFromAudioFile';
                
                stInfo = audioinfo(self.Filename);
                self.Fs = stInfo.SampleRate;
                self.SignalDimensions = [stInfo.TotalSamples, stInfo.NumChannels];
            else
                error(['Source is not recognized!', ...
                    'Pass a signal vector/matrix',...
                    'or a path to an audio file (e.g. C:/.../audio.wav)']);
            end
        end
        
        initiateBlockIndex(self);
        
        self.IsFinished = false;
    end
    
    function [data] = getBlock(self)
        % Essential method to optain a current block of the audio signal
        
        if self.IsFinished,
            warning('The last data block has been returned. No more data left!');
            data = [];
            return;
        end
        
        
        % --------------------------------------------------------------- %
        if self.ThisBlock(end) < self.SignalDimensions(1),
            data = self.(self.RetrievalFun)(self.ThisBlock);
            data = data(:,self.IdxChannels);
        else
            data = self.(self.RetrievalFun)(self.ThisBlock(1):self.SignalDimensions(1));
            data = [data(:, self.IdxChannels); zeros(self.RemainingSamples, length(self.IdxChannels))];
            
            self.IsFinished = true;
        end
        data = data .* (self.WindowFunction(self.Blocklength) * ones(1,length(self.IdxChannels)));
        % --------------------------------------------------------------- %
        
        self.ThisBlock = self.ThisBlock + self.Frameshift;
    end
    
    function signalOut = unbuffer(self, signalBlocks)
        [BlocklengthIn, numBlocksIn] = size(signalBlocks);
        
        if numBlocksIn < self.NumBlocks || BlocklengthIn < self.Blocklength,
            error(['Use this function only with data retrievend from ',...
                'this class. Block length and/or number of data blocks ',...
                'does not correspond to the class'' properties!']);
        end
        
        signalBlocks = diag(sparse(self.WindowFunction(self.Blocklength))) * signalBlocks;
        
        blockIndices = 1:self.Blocklength;
        
        signalOut = zeros(self.LengthPaddedSignal,1);
        for iFrame = 1:self.NumBlocks,
            signalOut(blockIndices) = ...
                signalOut(blockIndices) + signalBlocks(:,iFrame);
            
            blockIndices = blockIndices + self.Frameshift;
        end
        
        signalOut = signalOut(1:self.SignalDimensions(1));
    end
    
    
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%% Setter / Getter %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    function bl = get.Blocklength(self)
        bl = round(self.BlocklengthSec * self.Fs);
    end
    
    function ol = get.Overlap(self)
        ol = round(self.Blocklength * self.OverlapRatio);
    end
    
    function fs = get.Frameshift(self)
        fs = self.Blocklength - self.Overlap;
    end

    function nb = get.NumBlocks(self)
        nb = max(1,ceil((self.SignalDimensions(1) - self.Overlap) / self.Frameshift));
    end

    function len = get.LengthPaddedSignal(self)
        len = self.NumBlocks * self.Frameshift + self.Overlap;
    end
    
    function rs = get.RemainingSamples(self)
        rs = self.LengthPaddedSignal - self.SignalDimensions(1);
    end


    
    
    function [] = set.BlocklengthSec(self, BlocklengthIn)
        validateattributes(BlocklengthIn, {'numeric'}, ...
            {'nonzero', 'nonnegative', '<=', self.SignalDimensions(1)}); %#ok<MCSUP>
        
        self.BlocklengthSec = BlocklengthIn;
        initiateBlockIndex(self);
    end
    
    function [] = set.OverlapRatio(self, overlapIn)
        validateattributes(overlapIn, {'numeric'}, {'nonnegative', '<', 1});
        
        self.OverlapRatio = overlapIn;
    end
    
    function [] = set.WindowFunction(self,WinIn)
        validateattributes(WinIn, {'function_handle', 'char'}, {});
        
        if isa(WinIn,'function_handle'),
            self.WindowFunction = WinIn;
        elseif ischar(WinIn),
            self.WindowFunction = str2func(WinIn);
        end
    end
end



methods (Access = private)    
    function [] = initiateBlockIndex(self)
        self.ThisBlock = 1 : self.Blocklength;
    end
    
    function [vData] = cutIntoBlocks(self,vBlockIdx)
        vData = self.Signal(vBlockIdx,:);
    end
    
    function [vData] = readFromAudioFile(self,vBlockIdx)
        vData = audioread(self.Filename,[vBlockIdx(1), vBlockIdx(end)]);
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
