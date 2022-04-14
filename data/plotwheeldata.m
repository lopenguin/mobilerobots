function  plotwheeldata(bagfilename)
%
%   plotwheeldata(bagfilename)
%
%   Plot the wheel command, desired, and actual for left/right wheels.
%   If 'bagfilename' is not given or given as 'latest', use the most
%   recent bag file.
%

% If no bagfile is specified, use the most recent.
if (~exist('bagfilename') || strcmp(bagfilename, 'latest'))
    bagfilename = latestbagfilename();
end

% Proceed with each wheel
plotwheel(1, bagfilename, 'leftwheel');
plotwheel(2, bagfilename, 'rightwheel');
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function  plotwheel(fig, bagfilename, joint)
%
%   plotwheel(fig, bagfilename, joint)
%
%   Plot the data in the name figure, from the named bag file, for the
%   named joint.
%

% Function to grab the data.
function [t, p, v, e, name] = data(topic)
    try
        msgs = rosbagmsgs(bagfilename, topic);
        [t, p, v, e, name] = jointstatedata(msgs, joint);
    catch
        [t, p, v, e, name] = deal([], [], [], [], {});
    end
end

% Read the data.
[tc, pc, vc, ec, name] = data('/wheel_command');
[td, pd, vd, ed, name] = data('/wheel_desired');
[ta, pa, va, ea, name] = data('/wheel_state');  

% Plot.
figure(fig);
clf;

% Plot.
ax(1) = subplot(2,1,1);
hold on;
if ~isempty(pc), plot(tc,pc,'bo--','LineWidth',2,'DisplayName','Command'); end
if ~isempty(pd), plot(td,pd,'ro:', 'LineWidth',2,'DisplayName','Desired'); end
if ~isempty(pa), plot(ta,pa,'gx-', 'LineWidth',2,'DisplayName','Actual');  end
grid on;
ylabel('Position (rad)');
title(['Data for ' joint]);
legend;

ax(2) = subplot(2,1,2);
if ~isempty(vc), plot(tc,vc,'bo--','LineWidth',2,'DisplayName','Command'); end
if ~isempty(vd), plot(td,vd,'ro:', 'LineWidth',2,'DisplayName','Desired'); end
if ~isempty(va), plot(ta,va,'gx-', 'LineWidth',2,'DisplayName','Actual');  end
grid on;
ylabel('Velocity (rad/sec)');
xlabel('Time (sec)');

linkaxes(ax, 'x');

% Name the Figure and span the full 8.5x11 page.
set(gcf, 'Name',          'Joint Data');
set(gcf, 'PaperPosition', [0.25 0.25 8.00 10.50]);

% Return to the top subplot, so subsequent title()'s go here...
subplot(2,1,1);

end
