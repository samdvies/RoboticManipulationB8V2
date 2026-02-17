function varargout = angleConversion(mode, val, joint_idx)
% ANGLECONVERSION Converts between code angles (radians), degrees, and Dynamixel encoder values
%
% DYNAMIXEL XM430 CONVENTION:
%   Encoder 0    = 0° (Wizard)    = -180° from home
%   Encoder 1024 = 90° (Wizard)   = -90° from home
%   Encoder 2048 = 180° (Wizard)  = 0° (HOME)
%   Encoder 3072 = 270° (Wizard)  = +90° from home
%   Encoder 4095 = ~360° (Wizard) = +180° from home
%
% Operating Range (±90°):
%   Code: [-π/2, +π/2] radians
%   Encoder: [1024, 3072]
%
% USAGE MODES:
%
% 1. Convert radians to encoder:
%    encoder = angleConversion('rad2enc', radians)
%
% 2. Convert encoder to radians:
%    radians = angleConversion('enc2rad', encoder)
%
% 3. Convert degrees (from home) to encoder:
%    encoder = angleConversion('deg2enc', degrees)
%
% 4. Convert encoder to degrees (from home):
%    degrees = angleConversion('enc2deg', encoder)
%
% 5. Display conversion table:
%    angleConversion('table')
%
% Examples:
%    enc = angleConversion('rad2enc', pi/4)    % 45° → 2560
%    rad = angleConversion('enc2rad', 1024)    % → -π/2
%    angleConversion('table')                   % Print reference table
%
% Author: OpenManipulator-X FK Simulation
% Date: February 2026

    if nargin < 1
        printConversionTable();
        return;
    end
    
    if nargin < 3
        joint_idx = 0; % Default: No inversion
    end
    
    % INVERSION MAPPING:
    % Joint 2 (Shoulder) and Joint 3 (Elbow) need inversion
    % Code: +Angle = Up/Back
    % Motor: +Encoder = Down/Forward
    start_sign = 1;
    offset_deg = 0;
    
    if joint_idx == 2
        start_sign = 1;   % +Angle (Down) = +Encoder (Down)
        offset_deg = -90; % Hardware 2048 (Up) -> Kinematics -90 (Up)
    elseif joint_idx == 3
        start_sign = -1;  % +Angle (In) = -Encoder (In)
        offset_deg = -90; % Hardware 2048 (Forward) -> Kinematics -90 (Forward/L-Shape)
    end
    
    switch lower(mode)
        case 'rad2enc'
            % Radians (Kinematic) to Encoder
            rad = val;
            deg = rad2deg(rad);
            
            % Remove Offset first, then Invert?
            % Formula: EncDeg = (KinDeg - Offset) * Sign?
            % Let's check J2:
            % Desired Kin=-90 -> Enc=2048 (Deg 0)
            % (-90 - (-90)) * -1 = 0. Correct.
            % Desired Kin=0 (Horiz) -> (-90 + 90) = 0?
            % Wait. Up is -90. Horiz is 0. 
            % If Kin=0 -> (0 - (-90)) * -1 = -90 -> Enc 1024.
            % Does Enc 1024 mean Horizontal?
            % 2048=Up. +Enc=Down. 1024=Up-(-90)=Back?
            
            deg_enc = (deg - offset_deg) / start_sign;
            
            encoder = round((deg_enc / 360) * 4096) + 2048;
            encoder = max(0, min(4095, encoder));  % Clamp to valid range
            varargout{1} = encoder;
            
        case 'enc2rad'
            % Encoder to Radians (Kinematic)
            encoder = val;
            deg_enc = ((encoder - 2048) / 4096) * 360;
            
            % Apply Inversion then Offset
            deg = (deg_enc * start_sign) + offset_deg;
            
            varargout{1} = deg2rad(deg);
            
        case 'deg2enc'
            % Degrees from home to Encoder
            deg = val;
            encoder = round((deg / 360) * 4096) + 2048;
            encoder = max(0, min(4095, encoder));
            varargout{1} = encoder;
            
        case 'enc2deg'
            % Encoder to Degrees from home
            encoder = val;
            deg = ((encoder - 2048) / 4096) * 360;
            varargout{1} = deg;
            
        case 'table'
            printConversionTable();
            
        otherwise
            error('Unknown mode: %s. Use ''rad2enc'', ''enc2rad'', ''deg2enc'', ''enc2deg'', or ''table''.', mode);
    end
end

function printConversionTable()
    fprintf('\n');
    fprintf('╔══════════════════════════════════════════════════════════════╗\n');
    fprintf('║         DYNAMIXEL ANGLE CONVERSION REFERENCE                 ║\n');
    fprintf('╠══════════════════════════════════════════════════════════════╣\n');
    fprintf('║  Code (rad)  │  Code (deg)  │  Encoder  │  Wizard Display   ║\n');
    fprintf('╠══════════════════════════════════════════════════════════════╣\n');
    fprintf('║    -π/2      │    -90°      │   1024    │      90°          ║\n');
    fprintf('║    -π/4      │    -45°      │   1536    │     135°          ║\n');
    fprintf('║     0        │      0° ←HOME│   2048    │     180° ←HOME    ║\n');
    fprintf('║    +π/4      │    +45°      │   2560    │     225°          ║\n');
    fprintf('║    +π/2      │    +90°      │   3072    │     270°          ║\n');
    fprintf('╠══════════════════════════════════════════════════════════════╣\n');
    fprintf('║  OPERATING RANGE: Encoder [1024, 3072] = ±90° from home     ║\n');
    fprintf('╠══════════════════════════════════════════════════════════════╣\n');
    fprintf('║  Conversion: encoder = (deg/360)*4096 + 2048                ║\n');
    fprintf('║              deg = (encoder - 2048)/4096 * 360              ║\n');
    fprintf('╚══════════════════════════════════════════════════════════════╝\n');
    fprintf('\n');
end
