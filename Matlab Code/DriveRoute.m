function [new_i, new_j, map, new_heading] = DriveRoute(b, map, i, j, heading)
    %Reads in the route map and drives the rover into the next block
    
    if map(i, j) == 1
        return
    else
        map(i, j) = 1;
    end
    
    %Check if we have to move up
    if (j - 1 > 0 && map(i, j - 1) == -1)
        new_heading = 180;
        new_i = i;
        new_j = j - 1;

    elseif (i - 1 > 0 && map(i - 1, j) == -1)
        new_heading = 90;
        new_i = i - 1;
        new_j = j;
        
    elseif (i + 1 < 5 && map(i + 1, j) == -1)
        new_heading = 270;
        new_i = i + 1;
        new_j = j;
        
    elseif (j + 1 < 9 && map(i, j + 1) == -1)
        new_heading = 0;
        new_i = i;
        new_j = j + 1;
        
    end

    map(i, j) = 1;
    %We know the heading of where we want to go now. What sequence is required
    %to get there

    if (new_heading >= heading)   
        if (new_heading - heading == 90)
            %Turn left
            Direction = 'a';
            N = 1;
        elseif (new_heading - heading == 180)
            %Turn left twice
            Direction = 'a';
            N = 2;
        elseif (new_heading - heading == 270)
            %Turn right
            Direction = 'd';
            N = 1;
        else
            %No need to turn
            Direction = 'z';
            N = 0;
        end
        
    else
        if (heading - new_heading == 90)
            %Turn left
            Direction = 'd';
            N = 1;
        elseif (heading - new_heading == 180)
            %Turn left twice
            Direction = 'a';
            N = 2;
        elseif (heading - new_heading == 270)
            %Turn right
            Direction = 'a';
            N = 1;
        else
            %no turn
            Direction = 'z';
            N = 0;
        end
    end
    
    disp(i);
    disp(j);
    disp(Direction);
    disp(N);
    disp('--------');
    
    %Execute Turn Commands
    while (N > 0)
       Bluewrite(b, Direction, 1);
       N = N - 1;
    end
    
    %Move into the block
    Bluewrite(b, 'w', 1);

end
