function p = leg_parameters
% LEG_PARAMETERS Physical, electrical, and simulation parameters for the demo.
% Edit this file, then run build_leg_model followed by run_simulation.m.
% Values are illustrative assumptions, not calibrated hardware measurements.

p.M = 10;                  % kg, total moving mass; fixed base excluded
p.m = 0.5;                 % kg, leg mass; carriage mass is M-m
p.L = 0.16;                % m, leg length
p.width = 0.01;            % m, width and thickness of the uniform square rod
p.lc = p.L/2;              % m, joint-to-center-of-mass distance
p.J = p.m*(p.L^2+p.width^2)/12 + p.m*p.lc^2; % kg*m^2, inertia about the joint
p.q0 = pi/3;               % rad, initial 60 deg; 90 deg is vertical
p.g = 9.81;                % m/s^2, gravitational acceleration
p.torque = 8;              % N*m, constant target electromagnetic torque
p.b = 11;                  % N*m*s/rad, assumed joint damping for a slow demo
p.Kt = 1; p.Ke = 1;       % N*m/A and V*s/rad, direct drive
p.R = 1; p.Le = 0.01;     % Ohm and H, armature resistance and inductance
p.Vmax = 24;              % V, average drive voltage limit
p.Kp = 5; p.Ki = 100;     % current PI gains
p.kGround = 1e5;          % N/m, ground contact stiffness
p.dGround = 500;          % N*s/m, ground contact damping
p.transition = 1e-4;      % m, contact smoothing region
p.T = 3; p.Ts = 0.001;    % s, simulation duration and logging interval

% Loaded initial rest: precompress the ground and initialize support current.
p.z0 = p.L*sin(p.q0)-p.M*p.g/p.kGround;
p.I0 = (p.M*p.L-p.m*p.lc)*p.g*cos(p.q0)/p.Kt;
end
