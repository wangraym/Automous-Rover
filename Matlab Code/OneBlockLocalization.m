function [result, i, j] = OneBlockLocalization(map_ir, m_u, m_ir)
    
    %128 values, 4 for each square of the 2D array. Loops rows and then
    %column
    map_u = make_ultrasonic();
    
    result = 0;
    i = 0;
    j = 0;

    %Hard code: Check for the one square's IR reading that we can't
    %localize on
    
    if (m_ir(1) == 2 && m_ir(2) == 2 && m_ir(3) == 1 && m_ir(4) == 2)
        return
    end
    
    %Translate ultrasonic readings into cases
    
    %Loop through spots now
    
    for i = 1:4
        for j = 1:8
            count = (i-1)*32 + (j-1)*4;
            count2 = (i-1)*4 + 2;
            count3 = (j-1)*4 + 2;
            %ultrasonics first
            if ((map_u(count + 1) == m_u(1)) && (map_u(count + 2) == m_u(2)) && (map_u(count + 3) == m_u(3)) && (map_u(count + 4) == m_u(4)))
                %IRs
                if ((map_ir(count2, count3) == m_ir(1)) && (map_ir(count2, count3 + 1) == m_ir(2)) && (map_ir(count2 + 1, count3 + 1) == m_ir(3)) && (map_ir(count2 + 1, count3) == m_ir(4)))
                    result = 1;
                    return;
                end
            end
        end
    end
