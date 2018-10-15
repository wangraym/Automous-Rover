function [m_u, m_ir] = Start(s, str, expected)
    
    if nargin > 1
        fwrite(s,str);
        %pause(1);
    end
    
    while s.bytesAvailable < expected
        disp('Waiting for return');
        pause(1);
    end
    
    disp(s.BytesAvailable);
    m_ir = fread(s, (s.BytesAvailable)/2, 'int8');
    m_u = fread(s, (s.BytesAvailable), 'int8');
    
    %if counter < 70
        %Read successful
    %    output = char(fread(s,s.BytesAvailable)');
    %else
        %Function failed, try again
    %    output = -1;
    %end
    
end