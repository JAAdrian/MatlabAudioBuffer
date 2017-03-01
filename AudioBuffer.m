classdef AudioBuffer < matlab.System
%AUDIOBUFFER Class providing methods for convenient audio block processing
% -------------------------------------------------------------------------
% AudioBuffer produces an object providing methods to conveniently work
% with traditional audio block processing. As input can be used a long
% signal vector or a filename pointing to an audio file.
% The essential method `getBlock` returns one signal block at a time
% according to the desired parameters block length and overlap.
%
% The class has dependencies to the MATLAB Signal Processing Toolbox.
%
%
% AudioBuffer Properties:
%   SignalDimensions - Signal Dimensions
%   NumBlocks        - Number of Signal Blocks
%   BlockSize        - Block Length in Samples
%   SampleRate       - Sampling Frequency in Hz
%   BlockLengthSec   - Block Length in Seconds
%   OverlapRatio     - Ratio of Overlapping Samples
%   WindowFunction   - Name of the Window Function
%   IdxChannels      - Chosen Signal Channels
%
% AudioBuffer Methods:
%    getBlock - get a current block of audio data
%    WOLA - reconstruct the original signal via WOLA method
%
%
% Example:
% =========================================================================
%   obj = AudioBuffer(signal, sampleRate);
%   obj.BlockLengthSec = 32e-3;
%   obj.OverlapRatio = 0.5;
%   obj.WindowFunction = @(x) hann(x, 'periodic');
%
%   numBlocks = obj.NumBlocks;
%   blockSize = obj.BlockSize;
%
%   signalBlocks = zeros(blockSize, numBlocks);
%   for iBlock = 1:numBlocks
%       signalBlocks(:, iBlock) = obj.getBlock;
%   end
%
%   reconstructedSignal = obj.WOLA(signalBlocks);
%
%   norm(signal - reconstructedSignal)^2
%
%   ans =
%
%     0.0021
% =========================================================================
%
% See also: STR2FUNC, AUDIOREAD, AUDIOINFO
%
% Author:  J.-A. Adrian (JA) <jensalrik.adrian AT gmail.com>
% Date  :  20-Mar-2015 13:04:55
%

% Version: v0.1   initial release, 20-Mar-2015 (JA)
%          v1.0   added input checks, improve documentation, 16-Mar-2016
%                 (JA) 
%          v1.0.1 fix ugly commas in statements
%          v2.0   refactor the class inheriting from matlab.System,
%                 19-Feb-2017 (JA)
%          v2.0.1 updated README, 19-Feb-2017 (JA)
%          v2.0.2 changed affiliation, 01-Mar-2017 (JA)
% 


properties (Access = private)
    Signal;

    Filename;
    RetrievalFun;

    ThisBlock;

    IsFinished = false;
end

properties (Access = private, Dependent)
    Overlap;
    FrameShift;
    LengthPaddedSignal;
    RemainingSamples;
end

properties (SetAccess = private)
%SampleRate Sampling Frequency in Hz
%   Sampling rate which has been passed or read from the chosen audio file.
    SampleRate;

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

%BlockSize Block Length in Samples.
%   Block length in samples based on the chosen length in seconds and the
%   sampling frequency.
    BlockSize;
end

properties (Nontunable)
%BlockLengthSec Block Length in Seconds.
%   Block length in seconds. The default is 30 ms as it is usual in speech
%   processing.
    BlockLengthSec = 30e-3;

%OverlapRatio Ratio of Overlapping Samples
%   Overlap given as ratio re. to the block length. The default is 0,
%   i.e. 0 * 100%, due to the default rectangular window.
    OverlapRatio = 0;

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
    function self = AudioBuffer(Source, SampleRate, varargin)
    % Constructor of the AudioBuffer Class.
    %
    % Usage: obj = AudioBuffer(Source, SampleRate)
    %
    % Inputs:  ------------------
    %       Source - Either a signal vector/matrix or a filename
    %                pointing to an audio file
    %       SampleRate - Sampling rate in Hz
    %
    % Outputs: ------------------
    %       obj - AudioBuffer object
    %

        if nargin
            if isnumeric(Source)        % 'Source' is a data vector/matrix
                self.Signal       = Source;
                self.RetrievalFun = @(blockIdx) self.DecomposeIntoBlocks(blockIdx);
                self.SampleRate   = SampleRate;

                self.SignalDimensions = size(self.Signal);
                
            elseif ischar(Source)           % 'Source' is a path to a file
                assert(logical(exist(Source, 'file')), ...
                    'Path to audio file does not exist.' ...
                    );
                
                self.Filename     = Source;
                self.RetrievalFun = @(blockIdx) self.readFromAudioFile(blockIdx);

                stInfo          = audioinfo(self.Filename);
                self.SampleRate = stInfo.SampleRate;

                self.SignalDimensions = [stInfo.TotalSamples, stInfo.NumChannels];
            
            else
                error(['Source is not recognized!', ...
                    'Pass a signal vector/matrix', ...
                    'or a path to an audio file (e.g. C:/.../audio.wav)']);
            end
        end
        
        self.setProperties(nargin - 2, varargin{:});
    end
    

    function [signalOut] = WOLA(self, signalBlocks)
    % Reconstruct the decomposed signal using the weighted-overlap-add
    % (WOLA) method using the parameters of the AudioBuffer object.
    % This method is useful if the blocked data are altered outside the
    % class and it is desired to reconstruct a signal vector. Only use
    % single channel data with this function! For multichannel data
    % call this function multiple times using the respective channel
    % data.
    %
    % Usage: [signalOut] = WOLA(obj, signalBlocks)
    %
    % Inputs:  ------------------
    %       obj          - AudioBuffer object
    %       signalBlocks - Signal blocks matrix with dimensions
    %                      BlockSize x NumBlocks
    %
    % Outputs: ------------------
    %       signalOut - Reconstructed signal vector with the length of
    %                   the original signal in samples
    %

        validateattributes(signalBlocks, {'numeric'}, ...
            {'2d', 'size', [self.BlockSize, self.NumBlocks]}...
            );

        [BlockSizeIn, numBlocksIn] = size(signalBlocks);

        if numBlocksIn < self.NumBlocks || BlockSizeIn < self.BlockSize
            error(['Use this function only with data retrievend from ',...
                'this class. Block length and/or number of data blocks ',...
                'does not correspond to the class'' properties!']);
        end

        windowFunction = self.WindowFunction(self.BlockSize);

        % Apply synthesis window.
        signalBlocks = diag(sparse(windowFunction)) * signalBlocks;

        % Initialize block indices.
        blockIndices = 1:self.BlockSize;

        % Begin WOLA by adding overlapping weighted blocks.
        signalOut = zeros(self.LengthPaddedSignal,1);
        for iFrame = 1:self.NumBlocks
            signalOut(blockIndices) = ...
                signalOut(blockIndices) + signalBlocks(:,iFrame);

            % Increment block indices.
            blockIndices = blockIndices + self.FrameShift;
        end

        % For perfect reconstruction the constant-overlap-add (COLA) term
        % has to be incorporated. Especially useful for overlaps != 0.5
        normFactor = self.FrameShift / norm(windowFunction)^2;

        signalOut = normFactor * signalOut;

        % Limit the signal length to the original length if zero padding
        % was applied.
        signalOut = signalOut(1:self.SignalDimensions(1));
    end



    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%% Setter / Getter %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    function bl = get.BlockSize(self)
        bl = round(self.BlockLengthSec * self.SampleRate);
    end

    function ol = get.Overlap(self)
        ol = round(self.BlockSize * self.OverlapRatio);
    end

    function SampleRate = get.FrameShift(self)
        SampleRate = self.BlockSize - self.Overlap;
    end

    function nb = get.NumBlocks(self)
        nb = max(1,ceil((self.SignalDimensions(1) - self.Overlap) / self.FrameShift));
    end

    function len = get.LengthPaddedSignal(self)
        len = self.NumBlocks * self.FrameShift + self.Overlap;
    end

    function rs = get.RemainingSamples(self)
        rs = self.LengthPaddedSignal - self.SignalDimensions(1);
    end




    function [] = set.BlockLengthSec(self, BlockSizeIn)
        validateattributes(BlockSizeIn, {'numeric'}, ...
            {'nonzero', 'nonnegative', '<=', self.SignalDimensions(1)}); %#ok<MCSUP>

        self.BlockLengthSec = BlockSizeIn;
        initiateBlockIndex(self);
    end

    function [] = set.OverlapRatio(self, overlapIn)
        validateattributes(...
            overlapIn, ...
            {'numeric'}, ...
            {'scalar', 'nonnegative', '<', 1}...
            );

        self.OverlapRatio = overlapIn;
    end

    function [] = set.WindowFunction(self,WinIn)
        validateattributes(WinIn, {'function_handle', 'char'}, {});

        if isa(WinIn,'function_handle')
            self.WindowFunction = WinIn;
        elseif ischar(WinIn)
            self.WindowFunction = str2func(WinIn);
        end
    end
end


methods (Access = protected)
    function [] = setupImpl(self)
        % set block index to 1.
        initiateBlockIndex(self);

        % initialize the IsFinished flag.
        self.IsFinished = false;
    end

    function [data] = stepImpl(self)
    % Essential method to optain a current block of the audio signal
    %
    % Usage: [data] = getBlock(obj)
    %
    % Inputs:  ------------------
    %       obj - AudioBuffer object
    %
    % Outputs: ------------------
    %       data - next signal block with desired size and following
    %              desired overlap
    %

        % Stop and warn if the last block is already put out.
        if self.IsFinished
            warning('The last data block has been returned. No more data left!');
            data = [];
            return;
        end


        % --------------------------------------------------------------- %
        % Return data depending on whether the source is a signal
        % vector/matrix or an audio file. Pad the last block with zeros if
        % the number of remaining samples is smaller than the desired block
        % size.

        if self.ThisBlock(end) < self.SignalDimensions(1)
            data = self.RetrievalFun(self.ThisBlock);
            data = data(:, self.IdxChannels);
        else
            data = self.RetrievalFun(self.ThisBlock(1):self.SignalDimensions(1));
            data = [
                data(:, self.IdxChannels);
                zeros(self.RemainingSamples, length(self.IdxChannels))
                ];

            % This was the last block.
            self.IsFinished = true;
        end

        % Apply window function to the current data vector/matrix and make
        % sure the window has the correct dimensions if dealing with number
        % of channels > 1.
        data = data .* (self.WindowFunction(self.BlockSize) * ones(1,length(self.IdxChannels)));
        % --------------------------------------------------------------- %

        % increment the block indices.
        self.ThisBlock = self.ThisBlock + self.FrameShift;
    end
end


methods (Access = private)
    function [] = initiateBlockIndex(self)
        % Sets block indices to the first block

        self.ThisBlock = 1 : self.BlockSize;
    end

    function [data] = DecomposeIntoBlocks(self, blockIdx)
        % Function to extract blocks from a signal vector/matrix

        data = self.Signal(blockIdx, :);
    end

    function [data] = readFromAudioFile(self, blockIdx)
        % Function to extract blocks from an audio file

        data = audioread(self.Filename, [blockIdx(1), blockIdx(end)]);
    end
end

end






% End of file: AudioBufferClass.m
