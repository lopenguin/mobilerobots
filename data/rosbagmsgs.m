function  msgs = rosbagmsgs(bagfilename, topicname)
%
%   msgs = rosbagmsgs(bagfilename, topicname)
%
%   Extract the messages of the named topic from the bagfile.  The
%   messages are returned as a struct array.  The structure contains
%   MessageType as well as the fields of the topic.
%

% Load the bag.
try
    bag = rosbag(bagfilename);
catch
    error(['Unable to open the bag file ''' bagfilename '''']);
end

% Isolate the specified topic.
topic = select(bag, 'Topic', topicname);
if (~topic.NumMessages)
    warning(['No messages under topic ''' topicname '''']);
end

% Convert the messages in the topic into structure array.
msgs = cell2mat(readMessages(topic, 'DataFormat', 'struct'));

end
