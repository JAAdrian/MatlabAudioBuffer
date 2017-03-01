MatlabAudioBuffer
==============================

This class aims at providing a convenient interface for working with block-wise (audio) signals. It can be used with single or multi-channel signals and handles audio files and signal vectors/matrices. It inherits from `matlab.System` and will, in a future release, be usable as a SIMULINK model. At the moment, some infos are missing to be included in SIMULINK.

Dependencies
-------------------------------

No dependencies to toolboxes.

The code has been tested with MATLAB R2015a/15b/16a/16b on Windows 7 and Xubuntu 15.10 and 16.04 LTS.


Installation
--------------------------------

Put the file `AudioBuffer.m` into your MATLAB path or the directory of your MATLAB project.

Usage
---------------------------------

The usage of the class is straight forward. The user can specify a signal vector or matrix with corresponding sampling rate or a filename. The supported file formats depend directly on those which the `audioread()` function of the user's MATLAB version supports. These include typically `wav`, `flac` and `mp3`.

The following example shows a typical usage of the class.

```matlab
obj = AudioBuffer(...
    signal, sampleRate, ...
    'BlockLengthSec', 32e-3, ...
    'OverlapRatio', 0.5, ...
    'IdxChannels', 1, ...
    'WindowFunction', @(x) hann(x, 'periodic') ...
    );
    
% or:
% obj = AudioBuffer( ...
%     filename, [], ...
%     'BlockLengthSec', 32e-3, ...
%     'OverlapRatio', 0.5, ...
%     'IdxChannels', 1, ...
%     'WindowFunction', @(x) hann(x, 'periodic') ...
%     );

numBlocks = obj.NumBlocks;
blockSize = obj.BlockSize;

signalBlocks = zeros(blockSize, numBlocks);
for iBlock = 1:numBlocks
    % iteratively return one signal block
    signalBlocks(:, iBlock) = obj.step();
end

% If desired, use the internal WOLA method to
% reconstruct the unblocked signal
reconstructedSignal = obj.WOLA(signalBlocks);

% this is the error between original
% and reconstructed signal.
norm(signal - reconstructedSignal)^2

% ans =
%
%      0.0021
```

License
---------------------------------

The code is distributed under BSD 3-Clause.
