function Values = GetSimData(out,str)
    % Extract values from logged simulation data by name
    % Output: columns are samples
    idx = find(strcmp(out.getElementNames,str));
    Values = out{idx}.Values.Data';
end

