ds = eeTrajectory;                     
sig = ds.get('ee_actual');             
ts = sig.Values;                       
trajXYZ = squeeze(ts.Data);            

if size(trajXYZ,2) ~= 3 && size(trajXYZ,1) == 3
    trajXYZ = trajXYZ.';               
end

t = ts.Time;                            

% Remove consecutive duplicates
diffs = [true; any(diff(trajXYZ,1,1) ~= 0, 2)];
trajXYZ = trajXYZ(diffs, :);
t = t(diffs);
