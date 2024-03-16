%% Plotting the robot's position 
clear ; clc ; close all


pose     = readtable("pose_data1.csv");
distance = readtable("distance_data1.csv");
bearing  = readtable("bearing_data1.csv");

x_bot = pose{:, 1};
y_bot = pose{:, 2};
theta = pose{:, 3};

speed = 5;
figure
for i = 1:length(x_bot)/speed
    clf
    hold on

    plot(x_bot(1:i*speed), y_bot(1:i*speed), ".-", "MarkerSize", 10)
    plot(x_bot(i*speed), y_bot(i*speed), "o", "LineWidth", 2, "MarkerSize", 10)

    xlabel("x [m]")
    ylabel("y [m]")
    title("Robot's position")

    axis equal
    axis([0, 10, 0, 10])
    grid on

    pause(0.0001)
end

% plot(x_bot, y_bot, "^")
%
% xlabel("x [m]")
% ylabel("y [m]")
%
% grid on
% axis equal

%% Plotting the robot's theta

figure

plot(theta)

xlabel("sample step")
ylabel("orientation [rad]")

grid on

