function [ vystup, extremes ] = Translation(taxel_pos, start_i, end_i, red_center )
%
%   Translácia taxelov podla cerbeneho stredu

    vystup = zeros(end_i - start_i, 3);
    trans = zeros(1,3);
    extremes = [1, 1, 1, -1, -1, -1];
    %start_i + 3 is old center
    trans(1,1) = taxel_pos(start_i + 3, 1) - red_center(1,1);
    trans(1,2) = taxel_pos(start_i + 3, 2) - red_center(1,2);
    trans(1,3) = taxel_pos(start_i + 3, 3) - red_center(1,3);
    
    for i= start_i : end_i,
        for j=1:3
            vystup(i - start_i + 1, j) = taxel_pos(i,j) - trans(1,j);
        end
    end
    
    for i= start_i : end_i,
        for j=1:3
            if vystup(i- start_i + 1, j) < extremes(1, j)
                extremes(1, j) = vystup(i- start_i + 1, j);
            end
            if vystup(i- start_i + 1, j) > extremes(1, j + 3)
                extremes(1, j + 3) = vystup(i- start_i + 1, j);
            end    
        end
    end

end

