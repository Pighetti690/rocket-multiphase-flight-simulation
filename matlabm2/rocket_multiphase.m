%% Multi-phase rocket flight simulation
% Ascent → Coast → Apogee detection → Parachute descent
% Rocket: fiberglass airframe, AeroTech F62T-6 motor
% Parameters extracted from OpenRocket flight simulation

clear; clc; close all;

%% === Rocket parameters (from OpenRocket) ===
m0      = 0.792;    % Initial mass with motor [kg]
m_dry   = 0.767;    % Mass after burnout [kg]
t_burn  = 0.99;     % Burn duration [s]
T_avg   = 56;       % Average thrust during burn [N]
A_ref   = 22.9e-4;  % Body reference area [m^2]
Cd_asc  = 0.75;     % Drag coefficient (ascent)

% Parachute
D_para  = 0.70;                 % Parachute diameter [m]
A_para  = pi*(D_para/2)^2;      % Parachute area [m^2]
Cd_para = 0.80;                 % Cd (hemispherical, OpenRocket default)

% Environment
g   = 9.81;    % Gravity [m/s^2]
rho = 1.225;   % Air density at sea level [kg/m^3]

%% === PHASE 1: ASCENT under thrust (0 → t_burn) ===
% State y = [altitude; vertical velocity]
% Mass decreases linearly during burn
mdot = (m0 - m_dry)/t_burn;
m_t  = @(t) m0 - mdot*t;

odefun_ascent = @(t,y) [ ...
    y(2); ...
    -g + (T_avg - 0.5*rho*y(2)*abs(y(2))*Cd_asc*A_ref)/m_t(t) ];

[t1, y1] = ode45(odefun_ascent, [0 t_burn], [0; 0]);

%% === PHASE 2: COAST (burnout → apogee) ===
% Ends when vertical velocity crosses zero (apogee event)
opts2 = odeset('Events', @apogee_event, 'RelTol',1e-6);
odefun_coast = @(t,y) [ ...
    y(2); ...
    -g - 0.5*rho*y(2)*abs(y(2))*Cd_asc*A_ref/m_dry ];

[t2, y2, t_apogee, y_apogee] = ode45(odefun_coast, [t_burn 20], y1(end,:), opts2);

fprintf('=== APOGEE DETECTED ===\n');
fprintf('   Time:     %.2f s\n', t_apogee);
fprintf('   Altitude: %.1f m\n', y_apogee(1));

%% === PHASE 3: PARACHUTE DESCENT (apogee → ground) ===
% Drag now dominated by parachute
opts3 = odeset('Events', @ground_event, 'RelTol',1e-6);
odefun_descent = @(t,y) [ ...
    y(2); ...
    -g - 0.5*rho*y(2)*abs(y(2))*Cd_para*A_para/m_dry ];

[t3, y3, t_ground] = ode45(odefun_descent, [t_apogee 120], y_apogee, opts3);

fprintf('\n=== GROUND CONTACT ===\n');
fprintf('   Time:              %.2f s\n', t_ground);
fprintf('   Impact velocity:   %.2f m/s\n', abs(y3(end,2)));
fprintf('   Total flight time: %.2f s\n', t_ground);

%% === Assemble full trajectory ===
t_all = [t1; t2; t3];
z_all = [y1(:,1); y2(:,1); y3(:,1)];
v_all = [y1(:,2); y2(:,2); y3(:,2)];

%% === Plots ===
figure('Name','Flight profile','Position',[100 100 1000 500]);

subplot(1,2,1);
plot(t_all, z_all, 'b-', 'LineWidth', 1.5); grid on; hold on;
xline(t_burn,   'k--', 'Burnout');
xline(t_apogee, 'k--', 'Parachute deploy');
xlabel('Time [s]'); ylabel('Altitude [m]');
title('Altitude vs time');

subplot(1,2,2);
plot(t_all, v_all, 'r-', 'LineWidth', 1.5); grid on; hold on;
yline(0, 'k:');
xline(t_burn,   'k--', 'Burnout');
xline(t_apogee, 'k--', 'Parachute');
xlabel('Time [s]'); ylabel('Vertical velocity [m/s]');
title('Velocity vs time');

%% === 2D animation + salvataggio GIF ===
figure('Name','Flight animation','Position',[100 100 500 800],'Color','white');

% Rimuovi timestamp duplicati per interp1
[t_unique, iu] = unique(t_all, 'stable');
z_unique = z_all(iu);
v_unique = v_all(iu);

dt_anim = 0.08;                            
t_anim = 0:dt_anim:t_unique(end);
z_anim = interp1(t_unique, z_unique, t_anim);
v_anim = interp1(t_unique, v_unique, t_anim);

gif_filename = 'rocket_flight.gif';
frame_delay  = 0.05;                       % secondi tra un frame e l'altro nella GIF

for k = 1:length(t_anim)
    t = t_anim(k); z = z_anim(k); v = v_anim(k);

    clf;
    axis([-25 25 -8 220]); grid on; hold on;
    xlabel('Horizontal [m]'); ylabel('Altitude [m]');

    fill([-25 25 25 -25], [-8 -8 0 0], [0.75 0.9 0.75], 'EdgeColor','none');

    body_h = 6;
    if t < t_apogee
        fill([-0.8 0.8 0.8 -0.8], [z z z+body_h z+body_h], [0.2 0.4 0.8]);
        fill([-0.8 0.8 0], [z+body_h z+body_h z+body_h+2], [0.8 0.2 0.2]);
        if t < t_burn
            fill([-0.5 0.5 0], [z z z-3], [1 0.6 0]);
        end
    else
        th = linspace(0,pi,30);
        fill(6*cos(th), z+body_h+6+3*sin(th), [1 0.9 0.3]);
        plot([-6 0], [z+body_h+6 z+body_h], 'k-');
        plot([ 6 0], [z+body_h+6 z+body_h], 'k-');
        plot([ 0 0], [z+body_h+6 z+body_h], 'k-');
        fill([-0.8 0.8 0.8 -0.8], [z z z+body_h z+body_h], [0.2 0.4 0.8]);
    end

    title(sprintf('t = %5.2f s  |  h = %6.1f m  |  v = %+6.2f m/s', t, z, v));
    drawnow;

    % --- Cattura frame e salvalo nella GIF ---
    frame = getframe(gcf);
    im = frame2im(frame);
    [imind, cm] = rgb2ind(im, 256);
    if k == 1
        imwrite(imind, cm, gif_filename, 'gif', 'Loopcount', inf, 'DelayTime', frame_delay);
    else
        imwrite(imind, cm, gif_filename, 'gif', 'WriteMode', 'append', 'DelayTime', frame_delay);
    end
end

fprintf('\nGIF salvata come: %s\n', gif_filename);
fprintf('Path completo: %s\n', fullfile(pwd, gif_filename));

%% === Local event functions ===
function [value, isterminal, direction] = apogee_event(~, y)
    value = y(2);      % vertical velocity
    isterminal = 1;    % stop the integration
    direction = -1;    % crossing zero going down
end

function [value, isterminal, direction] = ground_event(~, y)
    value = y(1);      % altitude
    isterminal = 1;
    direction = -1;
end