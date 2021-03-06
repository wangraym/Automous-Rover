function [map] = MazeSolve3(map, row_s, col_s, row_f, col_f)
    
    map(row_s, col_s) = -1;

    %FOR LOADING ZONEs on right side of maps

    %Take in array, starting coordinates (_s variables), and destination
    %coordinates (_f variables)
    
    %Assuming the following in the map:
    %1 is open space
    %-1 is marked off path
    %0 is a wall
    %The map is a 4 x 8 array
    
    %Look right, down, up, then left
    
    if (col_s + 1) < 9 && map(row_s, col_s + 1) > 0
        newrow_s = row_s;
        newcol_s = col_s + 1;
        map(newrow_s, newcol_s) = -1;
        
    elseif (row_s + 1) < 5 && map(row_s + 1, col_s) > 0
        newrow_s = row_s + 1;
        newcol_s = col_s;
        map(newrow_s, newcol_s) = -1;
        
    elseif (row_s - 1) > 0 && map(row_s - 1, col_s) > 0
        newrow_s = row_s - 1;
        newcol_s = col_s;
        map(newrow_s, newcol_s) = -1;
        
    elseif (col_s - 1) > 0 && map(row_s, col_s - 1) > 0
        newrow_s = row_s;
        newcol_s = col_s - 1;
        map(newrow_s, newcol_s) = -1;
        
    end 
    
    if newrow_s == row_f && newcol_s == col_f
        return
        
    else
        map = MazeSolve3(map, newrow_s, newcol_s, row_f, col_f);
        
    end
    

end