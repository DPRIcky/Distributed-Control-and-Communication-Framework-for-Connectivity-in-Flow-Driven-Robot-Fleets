function plot_velocity(times, velocities)
    figure(2);
    clf;
    plot(times, velocities, 'k', 'LineWidth', 2);
    title('Velocity vs Time');
    xlabel('Time (s)');
    ylabel('Velocity (m/s)');
    xlim([0, max(times)]);
    ylim([0, 1.5]);
    grid on;
    drawnow;
end
