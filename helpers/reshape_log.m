function new_log = reshape_log(old_log)
%RESHAPE_LOG Reshaping Simulink log tensor

    dims = size(old_log);
    nd = ndims(old_log);
        
    if nd == 3
        % Lambda tensor case: [4N x 1 x M]
        % e.g.: 10 x 1 x 401  per N=5, M=401
        dim1 = dims(1);   % = 4N
        M    = dims(3);   % n° steps
        N    = dim1 / 4;
    
        new_log = zeros(M, N, 4);
        for k = 1:M
            col = old_log(:,:,k);   % 4N x 1
            col = col(:).';          % 1 x 4N
            Xk  = reshape(col, 4, N).';  % N x 4
            new_log(k,:,:) = Xk;  % M x N x 4
        end
    
    elseif nd == 2
        % Classic x and v state vector dims: [M x 2N]
        M    = dims(1);
        dim2 = dims(2);   % = 2N
        N    = dim2 / 2;
    
        new_log = zeros(M, N, 2);
        for k = 1:M
            row = old_log(k,:);         % 1 x 2N
            Xk  = reshape(row, 2, N).';  % N x 2
            new_log(k,:,:) = Xk;  % M x N x 2
        end
    
    else
        error('Input log tensor has unexpected dimensions (%s).', mat2str(dims));
    end
end