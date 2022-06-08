function MLX_TO_M(relative_path_mlx,relative_path_m)
% MLX_TO_M Converts a LiveScript into a Script.
%   
%   MLX_TO_M(RELATIVE_PATH_MLX,RELATIVE_PATH_M)
% 
%   Example:
%   relative_path_mlx = '.\src\code.mlx'
%   relative_path_m = '.\src\code.m';

    matlab.internal.liveeditor.openAndConvert(which(relative_path_mlx),relative_path_m)

end

