%%
% Written by Mohamed Eid
clear
clc
%% Parameters
m = [0.1,0.3,0.2];
L = [0.1,0.4,0.3];
r1 = 0.025;
g = 9.81;
%% Trajectory
d = [0 0 0 0.5 0 0 0 10];
q0 = d(1); v0 = d(2); ac0 = d(3);
q1 = d(4); v1 = d(5); ac1 = d(6);
t0 = d(7); tf = d(8);
M = [ 1 t0 t0^2 t0^3   t0^4    t0^5;
      0 1  2*t0 3*t0^2 4*t0^3  5*t0^4;
      0 0  2    6*t0   12*t0^2 20*t0^3;
      1 tf tf^2 tf^3   tf^4    tf^5;
      0 1  2*tf 3*tf^2 4*tf^3  5*tf^4;
      0 0  2    6*tf   12*tf^2 20*tf^3];
b=[q0; v0; ac0; q1; v1; ac1];
a = M\b;
%% Simulation
Data = sim('RRR_Robot_Control');
f = figure('Name','First Joint','NumberTitle','off');
f.Position = [50 0 1500 800];
subplot (2, 3, 1)
plot (Data.position_1.Time, Data.position_1.Data)
grid
xlabel ('Time(sec)')
ylabel('Position')
legend('Desired Angle','Actual Angle')
subplot (2, 3, 2)
plot (Data.position_1.Time, Data.velocity_1.Data)
grid
xlabel ('Time(sec)')
ylabel('Velocity')
legend('Desired Velocity','Actual Velocity')
subplot (2, 3, 3)
plot (Data.position_1.Time, Data.acceleration_1.Data)
grid
xlabel ('Time(sec)')
ylabel('Acceleration')
legend('Desired Acceleration','Actual Acceleration')
subplot (2, 3, 4)
plot (Data.position_1.Time, Data.position_1.Data(:,1)-Data.position_1.Data(:,2))
grid
xlabel ('Time(sec)')
ylabel('Position Error')
subplot (2, 3, 5)
plot (Data.position_1.Time, Data.velocity_1.Data(:,1)-Data.velocity_1.Data(:,2))
grid
xlabel ('Time(sec)')
ylabel('Velocity Error')
subplot (2, 3, 6)
plot (Data.position_1.Time, Data.acceleration_1.Data(:,1)-Data.acceleration_1.Data(:,2))
grid
xlabel ('Time(sec)')
ylabel('Acceleration Error')

%Second Joint
f = figure('Name','Second Joint','NumberTitle','off');
f.Position = [50 0 1500 800];
subplot (2, 3, 1)
plot (Data.position_1.Time, Data.position_2.Data)
grid
xlabel ('Time(sec)')
ylabel('Position')
legend('Desired Angle','Actual Angle')
subplot (2, 3, 2)
plot (Data.position_1.Time, Data.velocity_2.Data)
grid
xlabel ('Time(sec)')
ylabel('Velocity')
legend('Desired Velocity','Actual Velocity')
subplot (2, 3, 3)
plot (Data.position_1.Time, Data.acceleration_2.Data)
grid
xlabel ('Time(sec)')
ylabel('Acceleration')
legend('Desired Acceleration','Actual Acceleration')
subplot (2, 3, 4)
plot (Data.position_1.Time, Data.position_2.Data(:,1)-Data.position_2.Data(:,2))
grid
xlabel ('Time(sec)')
ylabel('Position Error')
subplot (2, 3, 5)
plot (Data.position_1.Time, Data.velocity_2.Data(:,1)-Data.velocity_2.Data(:,2))
grid
xlabel ('Time(sec)')
ylabel('Velocity Error')
subplot (2, 3, 6)
plot (Data.position_1.Time, Data.acceleration_2.Data(:,1)-Data.acceleration_2.Data(:,2))
grid
xlabel ('Time(sec)')
ylabel('Acceleration Error')

%Third Joint
Data = sim('RRR_Robot_Control');
f = figure('Name','Third Joint','NumberTitle','off');
f.Position = [50 0 1500 800];
subplot (2, 3, 1)
plot (Data.position_1.Time, Data.position_3.Data)
grid
xlabel ('Time(sec)')
ylabel('Position')
legend('Desired Angle','Actual Angle')
subplot (2, 3, 2)
plot (Data.position_1.Time, Data.velocity_3.Data)
grid
xlabel ('Time(sec)')
ylabel('Velocity')
legend('Desired Velocity','Actual Velocity')
subplot (2, 3, 3)
plot (Data.position_1.Time, Data.acceleration_3.Data)
grid
xlabel ('Time(sec)')
ylabel('Acceleration')
legend('Desired Acceleration','Actual Acceleration')
subplot (2, 3, 4)
plot (Data.position_1.Time, Data.position_3.Data(:,1)-Data.position_3.Data(:,2))
grid
xlabel ('Time(sec)')
ylabel('Position Error')
subplot (2, 3, 5)
plot (Data.position_1.Time, Data.velocity_3.Data(:,1)-Data.velocity_3.Data(:,2))
grid
xlabel ('Time(sec)')
ylabel('Velocity Error')
subplot (2, 3, 6)
plot (Data.position_1.Time, Data.acceleration_3.Data(:,1)-Data.acceleration_3.Data(:,2))
grid
xlabel ('Time(sec)')
ylabel('Acceleration Error')