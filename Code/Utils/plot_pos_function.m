function [] = plot_pos_function(pos_vector, plot_title)
%close(gcf);
figure;
plot3(pos_vector(1,:), pos_vector(2,:), pos_vector(3,:));
xlabel('x');
ylabel('y');
zlabel('z');
title(plot_title);
grid on;
end

