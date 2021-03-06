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
plotwheel(1, bagfilename, 'gyro');
% plotwheel(2, bagfilename, 'gyro');
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
%[tc, pc, vc, ec, name] = data('/wheel_command');
[td, pd, vd, ed, name] = data('/wheel_desired');
[ta, pa, va, ea, name] = data('/wheel_state');  

% Plot.
figure(fig);
clf;

figrows = 2;
figcols = 1;

% % Plot.
% ax(1) = subplot(figrows,figcols,1);
% hold on;
% %if ~isempty(pc), plot(tc,pc,'--','LineWidth',1.5,'DisplayName','Command'); end
% if ~isempty(pd), plot(td,pd,':', 'LineWidth',1.5,'DisplayName','Encoder'); end
% if ~isempty(pa), plot(ta,pa,'-', 'LineWidth',1.5,'DisplayName','Gyro');  end
% grid on;
% ylabel('Position (rad)');
% %sgtitle(['Data for ' joint]);

% legend;
% 
% ax(2) = subplot(figrows,figcols,2);
% hold on
% if ~isempty(vc), plot(tc,vc,'--','LineWidth',1.5,'DisplayName','Command'); end


if ~isempty(vd), plot(td,-vd,':', 'LineWidth',1.5,'DisplayName','Encoder'); end
hold on
if ~isempty(va), plot(ta,va,'-', 'LineWidth',1.5,'DisplayName','Gyro');  end
grid on;
ylabel('Velocity (rad/sec)');
xlabel('Time (sec)');

legend;

title("Encoder vs Gyro During Spin")

% ax(3) = subplot(figrows,figcols,3);
% grid on;
% plot(va,ea,'-','LineWidth',1.5,'DisplayName','PWM');
% ylabel('PWM Command');
% xlabel('Velocity (rad/sec)')
% hold on


% uptimes = find(ta>3.57 & ta<8.03);
% downtimes = find(ta>13.13 & ta<17.72);
% 
% rampup = fitlm(va(uptimes),ea(uptimes));
% rampdown = fitlm(va(downtimes),ea(downtimes));
% 
% predup = va*rampup.Coefficients.Estimate(2) + rampup.Coefficients.Estimate(1);
% preddown = va*rampdown.Coefficients.Estimate(2)+ rampdown.Coefficients.Estimate(1);
% 
% plot(va,predup,'--');
% plot(va,preddown,'--');
% 
% slopeup = rampup.Coefficients.Estimate(2)
% slopedown = rampdown.Coefficients.Estimate(2)
% 
% avgslope = (slopeup+slopedown)/2

linkaxes(ax, 'x');

% Name the Figure and span the full 8.5x11 page.
set(gcf, 'Name',          'Joint Data');
set(gcf, 'PaperPosition', [0.25 0.25 8.00 5]);

% Return to the top subplot, so subsequent title()'s go here...
subplot(figrows,figcols,1);

sgtitle("PWM to Motor Speed");

end
