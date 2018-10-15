%------------Initialization------------------------------------------------

%Initialize Bluetooth
b = Bluetooth('ROB9', 1);
fopen(b);
    %Case 1 is (3, 3)
    %Case 2 is (1, 6)
    %Case 3 is (1, 8)
    %Case 4 is (4, 8)
drop_case = 1;
map = [1 1 1 1 0 1 0 1; 1 1 0 1 1 1 1 1; 1 0 1 0 0 1 0 1; 1 1 1 1 1 1 0 1];
Bluewrite(b, 'l', 1);

%initalization of the world
dim1 = 32; dim2 = 16; 
locationindex = reshape(1:dim1*dim2,dim1,dim2)'; 
n = numel(locationindex);
rand('twister',5489);
bw = reshape(randi([0 1],n,1),dim2,dim1); %0 = black, 1 = white
heading = 90; %Start with robot faceing 'w' always, 0 is 'd'

%make blocks
M = zeros(size(bw));
Blocks = [2, 3; 3, 2; 4, 3; 5, 1; 5, 3; 7, 1; 7, 3; 7, 4;];
for xx = 1:size(Blocks,1),
	x = Blocks(xx,1); y = Blocks(xx,2);
	M(1+(y-1)*4:(y-1)*4+4, 1+(x-1)*4:(x-1)*4+4) = 1;
end
M = [ones(dim2,1) M ones(dim2,1)];
M = [ones(1, dim1+2); M; ones(1, dim1+2)];

%create mask for blocks
M = abs(M-1);
M = M(2:end-1, 2:end-1);

final = (bw+1).*(M);

%------------Localization--------------------------------------------------

%Get Localization readings
[m_u, m_ir] = Start(b, 's', 8);
m_u = m_u.';
m_ir = m_ir.';

%Localize
[result, local_row, local_col] = OneBlockLocalization(final, m_u, m_ir);

%Try again if it fails
while (result == 0 && local_row == 4 && local_col == 8)
    [m_u, m_ir] = Start(b, 's', 8);
    [result, local_row, local_col] = OneBlockLocalization(final, m_u, m_ir);
end

%In the case we are in the bad two blocks, move left and read front sensor
if (result == 0 && local_row == 0)
    response = Bluewrite(b, 'a', 1);
    if (response == 0)
        wall = Bluewrite(b, 'w', 1);
        %Deduce where we are
        if (wall == 1)
            local_col = 1;
            local_row = 4;
            response = Bluewrite(b, 'd', 1);
        else
            local_col = 4;
            local_row = 4;
            heading = 180;
        end
    end
end

Bluewrite(b, 'n', 1);

%------------Drive to Loading Zone-----------------------------------------

%Decide on which block to enter the loading zone from
if ((local_row > 2 && local_col < 7) || (local_row == 1 && local_col == 6))
    i_dest = 3;
    j_dest = 1;
else
    i_dest = 1;
    j_dest = 3;
end

%A few hard coded cases we want to preset the route for
if ((local_row == 1 && local_col == 6))
    map = [1 1 1 1 0 -1 0 1; 1 1 0 1 1 -1 1 1; -1 0 1 0 0 -1 0 1; -1 -1 -1 -1 -1 -1 0 1];
    
elseif ((local_row == 3 && local_col == 6))
    map = [1 1 1 1 0 1 0 1; 1 1 0 1 1 1 1 1; -1 0 1 0 0 -1 0 1; -1 -1 -1 -1 -1 -1 0 1];

%Any other case is fine
else
    map(local_row, local_col) = -1;
    map = MazeSolve(map, local_row, local_col, i_dest, j_dest);
end

%Move to loading zone
while (local_row ~= i_dest || local_col ~= j_dest)
   [local_row, local_col, map, heading] = DriveRoute(b, map, local_row, local_col, heading);
end

Bluewrite(b, 'o', 1);

%------------Pick up Block-------------------------------------------------

%Use Gripper
Bluewrite(b, 'g', 1);

if (heading == 90)
    local_row = local_row - 1;
else
    local_col = local_col - 1;
end

Bluewrite(b, 'p', 1);

%------------Drive to Dropoff Zone-----------------------------------------

%Plan route away from loading zone

if (local_col == 1 && local_row == 2 && drop_case ~= 1)
    Bluewrite(b, 'w', 1);
    local_row = 1;
    local_col = 1;
    heading = 90;
end

map = [1 1 1 1 0 1 0 1; 1 1 0 1 1 1 1 1; 1 0 1 0 0 1 0 1; 1 1 1 1 1 1 0 1];

if (drop_case == 1)
    %Drop off at (3, 3)
    i_dest = 3;
    j_dest = 3;
    map = MazeSolve2(map, local_row, local_col);
   
elseif (drop_case == 2)
    %Drop off at (1, 6), planning route to middle spot 2, 6
    map = MazeSolve3(map, local_row, local_col, 2, 6);
    i_dest = 2;
    j_dest = 6;
    
else
    %Drop off in column 8, planning route to spot 2, 8
    map = MazeSolve3(map, local_row, local_col, 2, 8);
    i_dest = 2;
    j_dest = 8;
    
end

%Move to dropoff
while (local_row ~= i_dest || local_col ~= j_dest)
    disp("Dropoff");
   [local_row, local_col, map, heading] = DriveRoute(b, map, local_row, local_col, heading);
end

%For Cases 2-4, need an extra bit of hard code to move to correct spot
if (drop_case == 2)
    Bluewrite(b, 'a', 1);
    Bluewrite(b, 'w', 1);

elseif (drop_case == 3)
    Bluewrite(b, 'a', 1);
    Bluewrite(b, 'w', 1);
    
elseif (drop_case == 4)
    Bluewrite(b, 'd', 1);
    Bluewrite(b, 'w', 1);
    Bluewrite(b, 'w', 1);
end

%------------Drop Block & Dance--------------------------------------------

Bluewrite(b, 'v', 1);
Bluewrite(b, 'x', 1);

%--------------------------------------------------------------------------

%Done