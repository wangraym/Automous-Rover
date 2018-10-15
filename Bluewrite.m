function output = Bluewrite(s,str, expected)
    %Write to the arduino and wait for an acknowledgement

    if nargin > 1
        fwrite(s,str);
        %pause(1);
    end
    
    while s.bytesAvailable < expected
    end
    
    output = fread(s,expected, 'int8');
    
end