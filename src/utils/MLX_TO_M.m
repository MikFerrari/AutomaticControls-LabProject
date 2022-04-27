function MLX_TO_M(relative_path_mlx,relative_path_m)
%%MLX_TO_M(RELATIVE_PATH_MLX,RELATIVE_PATH_M)
% Example:
% relative_path_mlx = '.\src\identification\Identificazione.mlx'
% relative_path_m = '.\src\identification\Identificazione.m';

    matlab.internal.liveeditor.openAndConvert(which(relative_path_mlx),relative_path_m)

end

