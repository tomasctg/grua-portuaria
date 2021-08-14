function plot_scene(estado_barco, x_positions, hy_cont, hx_cont, deltax_cont, ysb, boat_under_water)

    %% Constantes
    % Valores de datoscarro.m que no pasan por la generaci�n de
    % trayectorias.
    xt0 = -20;
    
    %% L�mites del gr�fico
    xmin = -25;
    xmax = 20;
    ymin = -21;
    ymax = 45;
    
    %% Ploteo muelle
    delta = 1;
    x_muelle = abs(xmin);% + delta;
    y_muelle = abs(ymin) + delta;
    rectangle('Position', [xmin-delta ymin-delta x_muelle y_muelle], 'EdgeColor', [0.6 0.3 0.1], 'LineWidth', 3, 'FaceColor', [0.8 0.5 0.1])    
    hold on
    axis([xmin xmax ymin ymax])
    
    %% Ploteo barco
    barco_x = [0, 0, size(x_positions,2)*(hx_cont+deltax_cont)+deltax_cont, size(x_positions,2)*(hx_cont+deltax_cont)+deltax_cont];
    barco_y = [ysb, -boat_under_water-0.5, -boat_under_water-0.5, ysb];
    plot(barco_x, barco_y, 'LineWidth', 3, 'Color', [0.2 0.2 0.2])
    
    %% Ploteo mar
    %Parte izq barco 
    rectangle('Position',[-delta ymin-delta delta-0.1 y_muelle-delta],'FaceColor', 'Blue','LineWidth', 0.01)
    hold on
    rectangle('Position',[-0.1 ymin (size(x_positions,2)*(hx_cont+deltax_cont)+deltax_cont + 0.2)  (abs(ymin)-abs(boat_under_water)-0.8)],'FaceColor', 'Blue','LineWidth', 0.01)
    hold on
    rectangle('Position',[(size(x_positions,2)*(hx_cont+deltax_cont)+deltax_cont + 0.1) ymin-delta delta-0.1 y_muelle-delta],'FaceColor', 'Blue','LineWidth', 0.01)
    hold on
    %% Ploteo contenedores
%     for i = 1:size(x_positions,2)
%         for j = 1:estado_barco(i)
%             plot(x_positions(i), hy_cont*j - boat_under_water, 'Marker', 's', 'MarkerSize', 15,'LineWidth', 3, 'Color', 'r')
%             
%         end
%     end
    x_actual = 0;
    for i = 1:size(x_positions,2)
        y_actual = - boat_under_water;
        x_actual = x_positions(i) - hx_cont/2;
        for j = 1:estado_barco(i)
            rectangle('Position', [x_actual y_actual hx_cont hy_cont], 'EdgeColor', [0.9 0.05 0.1], 'LineWidth', 2, 'FaceColor', [1 0.5 0.5])
            y_actual = y_actual + hy_cont; 
            
        end
    end
    hold on
end