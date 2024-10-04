function signed_distance = signed_distance_function(x, y, theta, set_in, set_out, show_plot, gifFilename, firstFrame, delayTime)
    % Offset trajectories (feasible set boundaries)
    desired_trajectory = @(x_val) 0.75 * x_val.^2; % Scaled to fit (2,2) target
    offset_positive = @(x_val) desired_trajectory(x_val) + 0.2;
    offset_negative = @(x_val) desired_trajectory(x_val) - 0.2;
    
    % Calculate the feasible set boundaries for the current x
    upper_bound = offset_positive(x);
    lower_bound = offset_negative(x);

    % Check if the point is inside or outside the feasible bounds
    if y >= lower_bound && y <= upper_bound  % Inside feasible set
        % Compute distances to all points in set_in
        for rows = 1:length(set_in)
            norm_values(rows) = vecnorm([x; y; theta] - set_in(rows, :)');
        end
        signed_distance = -min(norm_values);
        if show_plot
            figure(2); clf; % Clear the figure
            plot(norm_values, 'o');
            yline(signed_distance, 'color', 'green', 'LineWidth', 2);
            ylabel('Distance to Points in Safe Set');
            xlabel('Point Index');
            title('In Feasible Set');
            drawnow; % Update the figure immediately
            % Save frame to GIF
            frame = getframe(gcf);
            im = frame2im(frame);
            [imind, cm] = rgb2ind(im, 256);
            if firstFrame
                imwrite(imind, cm, gifFilename, 'gif', 'Loopcount', inf, 'DelayTime', delayTime);
            else
                imwrite(imind, cm, gifFilename, 'gif', 'WriteMode', 'append', 'DelayTime', delayTime);
            end
        end
    else  % Outside feasible set
        % Compute distances to all points in set_out
        for rows = 1:length(set_out)
            norm_values(rows) = vecnorm([x; y; theta] - set_out(rows, :)');
        end
        signed_distance = min(norm_values);
        if show_plot
            figure(2); clf; % Clear the figure
            plot(norm_values, 'o');
            yline(signed_distance, 'color', 'green', 'LineWidth', 2);
            ylabel('Distance to Points in Unsafe Set');
            xlabel('Point Index');
            title('Out of Feasible Set');
            drawnow; % Update the figure immediately
            % Save frame to GIF
            frame = getframe(gcf);
            im = frame2im(frame);
            [imind, cm] = rgb2ind(im, 256);
            if firstFrame
                imwrite(imind, cm, gifFilename, 'gif', 'Loopcount', inf, 'DelayTime', delayTime);
            else
                imwrite(imind, cm, gifFilename, 'gif', 'WriteMode', 'append', 'DelayTime', delayTime);
            end
        end
    end
end
