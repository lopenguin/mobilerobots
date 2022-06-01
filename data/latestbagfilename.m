function bagfilename = latestbagfilename()
%
%   bagfilename = latestbagfilename()
%
%   Return the name of the latest bag file.  Error if there are not
%   bag files.
%

% Get a list of .bag files in the current folder.
d = dir('*.bag');

% Make sure we have at least one bag file.
if (~size(d,1))
    error('Unable to find a bag file');
end

% Find the most recently modified file
[~, idx] = max([d.datenum]);

% Use as the bag file.
bagfilename = d(idx).name;
disp(['Using bag file ''' bagfilename '''']);

end
