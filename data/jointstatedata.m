function  [t, pos, vel, eff, names] = jointstatedata(msgs, joint)
%
%   [t, pos, vel, eff, names] = jointstatedata(msgs, joint)
%
%   Extract the data from the given JointStates messages.  Time is
%   shifted to start at zero for the first message.  The return data
%   gives a column per time sample, and a row per joint (assuming data
%   is available).
%
%   If 'joint' is given and not 'all', plot only the joint given by an
%   index (number) or name (string).
%

% Double-check the type.
if (~strcmp(msgs(1).MessageType, 'sensor_msgs/JointState'))
    error(['Messages are not of type sensor_msgs/JointState']);
end

% Extract the names of the joints and double-check.
names = msgs(1).Name;
if (length(names) == 0)
    error(['Messages contain no joints']);
end

% Extract the time (converting from sec/nsec).
headers = [msgs(:).Header];
stamps  = [headers(:).Stamp];

sec  = double([stamps(:).Sec]);
nsec = double([stamps(:).Nsec]);

t = (sec - sec(1)) + 1e-9 * (nsec - nsec(1));

% Extract the msgs.
pos = double([msgs(:).Position]);
vel = double([msgs(:).Velocity]);
eff = double([msgs(:).Effort]);

% Potentially isolate a single joint.
if (exist('joint') && (~strcmp(joint, 'all')))
    % Check the number of joints (by data).
    Nnam = length(names);
    Npos = size(pos, 1);
    Nvel = size(vel, 1);
    Neff = size(eff, 1);
    Nmax = max([Nnam, Npos, Nvel, Neff]);

    % For a numeric joint specification.
    if (isnumeric(joint))
        if (numel(joint) ~= 1)
            error('Bad joint index specification');
        end
            
        % Grab the joint number and use as index.
        ind = floor(joint);
        if ((ind < 1) || (ind > Nmax))
            error(['Out of range joint index ' num2str(ind)]);
        end
        disp(['Using only joint index ' num2str(ind)]);

    % For a joint name specification.
    elseif (ischar(joint))
        % Find the index for the joint name.
        ind = find(strcmp(names, joint));
        if (isempty(ind))
            error(['Unable to isolate joint ''' joint ''''])
        end
        disp(['Using only joint ''' joint '''']);

    % Otherwise can't do anything.
    else
        error('Bad joint argument');
    end
    
    % Isolate the data.
    if (ind <= Nnam) names = names(ind); else names = {}; end
    if (ind <= Npos) pos   = pos(ind,:); else pos   = []; end
    if (ind <= Nvel) vel   = vel(ind,:); else vel   = []; end
    if (ind <= Neff) eff   = eff(ind,:); else eff   = []; end
end

end
