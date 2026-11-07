%% RUN_SIMULATION Run the physical model and export raw simulation signals.
% Requires MATLAB, Simulink, Simscape, and Simscape Multibody.
% Run directly with the supplied dc_leg.slx; rebuilding is optional.
clear; clc;
folder = fileparts(mfilename('fullpath'));
root = fileparts(folder);
modelFile = fullfile(folder,'dc_leg.slx');
assert(isfile(modelFile),'Run build_leg_model.m first.');
load_system(modelFile);
p = getVariable(get_param('dc_leg','ModelWorkspace'),'p');
out = sim('dc_leg','ReturnWorkspaceOutputs','on');

time_s = out.sim_data.Time;
x = out.sim_data.Data;
measurements = table(time_s,x(:,1),x(:,2),x(:,3),x(:,4),x(:,5),x(:,6),x(:,7), ...
    'VariableNames',{'time_s','current_A','angle_rad','velocity_rad_s', ...
    'acceleration_rad_s2','carriage_height_m','voltage_V','electromagnetic_torque_Nm'});

% Direct output of the Spatial Contact Force block, converted to newtons.
groundForce = table(out.F_ground.Time,out.F_ground.Data, ...
    'VariableNames',{'time_s','force_N'});

outputFolder = fullfile(root,'results');
if ~isfolder(outputFolder), mkdir(outputFolder); end
writetable(measurements,fullfile(outputFolder,'measurements.csv'));
writetable(groundForce,fullfile(outputFolder,'ground_force.csv'));
fid = fopen(fullfile(outputFolder,'parameters.json'),'w');
assert(fid~=-1,'Cannot write parameters.json.');
fprintf(fid,'%s\n',jsonencode(p,'PrettyPrint',true));
fclose(fid);

figure('Name','DC motor leg simulation','Color','w','Position',[100 100 1100 720]);
tiledlayout(2,2,'TileSpacing','compact');
nexttile; plot(groundForce.time_s,groundForce.force_N,'k','LineWidth',1.2);
xlabel('Time (s)'); ylabel('Ground support (N)'); title('Contact block output'); grid on;
nexttile; plot(time_s,rad2deg(measurements.angle_rad),'LineWidth',1.2);
yline(90,'--','Color',[.4 .4 .4]); xlabel('Time (s)'); ylabel('Angle (deg)');
title('Leg angle'); grid on;
nexttile; plot(time_s,measurements.current_A,'LineWidth',1.2);
yline(p.torque/p.Kt,'--','Color',[.4 .4 .4]); xlabel('Time (s)'); ylabel('Current (A)');
title('Armature current and target'); grid on;
nexttile; plot(time_s,measurements.carriage_height_m,'LineWidth',1.2);
xlabel('Time (s)'); ylabel('Joint height (m)'); title('Vertical carriage motion'); grid on;
sgtitle('DC motor single-leg simulation','Color','k');
set(findall(gcf,'Type','axes'),'Color','w','XColor','k','YColor','k','GridColor',[.65 .65 .65]);
set(findall(gcf,'Type','text'),'Color','k');
exportgraphics(gcf,fullfile(outputFolder,'simulation.png'),'Resolution',160);
fprintf('Exported %d samples over %.3f s to %s\n',height(measurements),time_s(end),outputFolder);
