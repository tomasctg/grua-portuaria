function [vyt,vxt,x_end,vxt_end,vyt_end,len] = cont_to_cont(estado_barco,posx_init,posy_init,posx_end,twistlocks)
    tic
    %FUNCION PARA OBTENER TRAYECTORIA ENTRE CONTAINERS
    
    %Datos
    dt = 1e-3;
    
    if(twistlocks)
        vy_max_aux=1.5;
    else
        vy_max_aux = 3;
    end
    
    %Parametros
    vy_max = vy_max_aux
    vx_max = 1.0;
    ay_max=1;
    ax_max=1;
    boat_wide = 6;
    boat_under_water = 10;
    hy_cont = 2.5;
    hx_cont = 2.44;
    deltax_cont = 0.2;
    ysb=12;
    safety_distance=5;
    
    %Determino la coordenada en x de cada columna
    x_positions = [hx_cont/2 + deltax_cont];
    posx_init_index = 1;
    for i=2:boat_wide
        x_positions(i)=(x_positions(i-1) + hx_cont + deltax_cont);
        if(abs(posx_init - x_positions(i)) <= 0.1)
            posx_init_index = i
        end
    end
   
    
    %Determino si la columna a la que deseo ir esta a la derecho o
    %izquierda
    
    if(x_positions(posx_end) > posx_init)
        
        %Determino columna con mayor cantidad de containers a la derecha de
        %donde estoy
        
        [max_height_colunm,max_height_colunm_index] = max(estado_barco(posx_init_index:posx_end));
        max_height_colunm=max_height_colunm*hy_cont - boat_under_water + safety_distance;
        
        while(true)    
                time_max_acel_y =vy_max/ay_max;
                y_aceled = (ay_max/2)*(time_max_acel_y^2);
                %Aca es donde especifican las distancia desde la posicion en la
                %direccion correspondiente donde estoy hasta el siguiente punto.
                %Son todos deplazamientos relativos, despues arma la trayectoria
                %sumando las condiciones inciales de cada punto.
                t_velcont_y = ((max_height_colunm - (posy_init))-(y_aceled*2))/vy_max;
                if(t_velcont_y>=0)
                    break;
                end
                %vy_max_sc = vy_max_sc-0.0001;
                vy_max = sqrt((max_height_colunm - (posy_init))*ay_max)-0.1;
       end

                %RESET VALUES
                vx_max = 1.0; %[m/s]
                vy_max = vy_max_aux;
                ay_max=1;
                ax_max=1;

                %Armado de las consignas de velocidad.
                %Desde punto 0 hasta punto 1:

                %Tiempo total de trayectoria
                t_total_0 = (time_max_acel_y*2)+t_velcont_y;
                t=0:dt:t_total_0;
                ay0_t = [];   
                %Determinacion de aceleraciones para cada instante de tiempo 
                for u=1:length(t)
                    if(t(u)<=time_max_acel_y)   
                        ay0_t(u)=1;
                    elseif((t(end)-t(u))<=time_max_acel_y)
                        ay0_t(u)=-1;
                    else
                        ay0_t(u)=0;
                    end
                end

                %Integracion acumulativa, obtengo el perfil de velocidad.
                dy0_t = cumtrapz(t,ay0_t);
                %Integracion acumulativa, obtengo el perfil de posicion. Solo para 
                %corroborar, el perfil que usamos es de velocidad.
                y0_t = posy_init + cumtrapz(t,dy0_t);
                %Vector de 0s, porque solo se mueve en y.
                dx0_t = 0*t;

                %ACA SE ARMA LA TRAYECTORIA
                trayectoria_dx = [dx0_t' , t'];
                %Negativo por la convencion de izaje.
                trayectoria_dy = [-dy0_t', t'];
                
        while(true)
            time_max_acel_x =(vx_max)/ax_max;
            %time_max_acel_x =(vx_max)/ax_max;
            x_aceled = (ax_max/2)*(time_max_acel_x^2);
            t_velcont_x = ((x_positions(posx_end)-posx_init) - 2*x_aceled)/(vx_max);
            %t_velcont_x = ((x_positions(posx_end)-posx_init) - 2*x_aceled)/(vx_max);
            if(t_velcont_x>=0)
                break;
            end
            %vx_max=vx_max-0.0001;
            vx_max = sqrt( ( x_positions(posx_end)-posx_init ) * ax_max ) - 0.1;
        end
        
        t_total_1 = (time_max_acel_x*2)+t_velcont_x;
        
        t=dt:dt:t_total_1;
      
        ax1_t = [];   

        for u=1:length(t)
            if(t(u)<=time_max_acel_x)   
                ax1_t(u)=1;
            elseif((t(end)-t(u))<=time_max_acel_x)
                ax1_t(u)=-1;
            else
                ax1_t(u)=0;
            end
        end


        t = t + t_total_0;
        dx1_t = cumtrapz(t,ax1_t);        
        dy1_t = dx1_t*0;
        trayectoria_dx =[trayectoria_dx; [dx1_t' , t']];
        trayectoria_dy =[trayectoria_dy; [dy1_t' , t']];
        
        while(true)    
            time_max_acel_y =vy_max/ay_max;
            y_aceled = (ay_max/2)*(time_max_acel_y^2);
            %Aca es donde especifican las distancia desde la posicion en la
            %direccion correspondiente donde estot hasta el siguiente punto.
            %Son todos deplazamientos relativos, despues arma la trayectoria
            %sumando las condiciones inciales de cada punto.
            t_velcont_y = ((y0_t(end)-(estado_barco(posx_end)*hy_cont - boat_under_water + safety_distance))-(y_aceled*2))/vy_max;
            if(t_velcont_y>=0)
            break;
            end
            vy_max=vy_max-0.01;
            end

            %RESET VALUES
            vx_max = 1.0; %[m/s]
            vy_max = vy_max_aux; 
            ay_max=1;
            ax_max=1;

            %Armado de las consignas de velocidad.
            %Desde punto 0 hasta punto 1:

            %Tiempo total de trayectoria
            t_total_end = (time_max_acel_y*2)+t_velcont_y;
            t=0:dt:t_total_end;
            ayend_t = [];   
            %Determinacion de aceleraciones para cada isntante de tiempo 
            for u=1:length(t)
                if(t(u)<=time_max_acel_y)   
                    ayend_t(u)=1;
                elseif((t(end)-t(u))<=time_max_acel_y)
                    ayend_t(u)=-1;
                else
                    ayend_t(u)=0;
                end
            end

            %Integracion acumulativa, obtengo el perfil de velocidad.
            dyend_t_y = cumtrapz(t,ayend_t);
            %Integracion acumulativa, obtengo el perfil de posicion. Solo para 
            %corroborar, el perfil que usamos es de velocidad.

            %Vector de 0s, porque solo se mueve en y.
            dxend_t_x = 0*t;

        %     Final de la trayectoria
            vxt_end= dxend_t_x';
            %Negativo por la convencion de izaje.
            vyt_end= dyend_t_y';
            
            vyt = trayectoria_dy(:,1);
            vxt = trayectoria_dx(:,1);
            x_end = x_positions(posx_end);
    else
        
        [max_height_colunm,max_height_colunm_index] = max(estado_barco(posx_end:posx_init_index));
        max_height_colunm=max_height_colunm*hy_cont - boat_under_water + safety_distance;
        
        while(true)    
                time_max_acel_y =vy_max/ay_max;
                y_aceled = (ay_max/2)*(time_max_acel_y^2);
                %Aca es donde especifican las distancia desde la posicion en la
                %direccion correspondiente donde estoy hasta el siguiente punto.
                %Son todos deplazamientos relativos, despues arma la trayectoria
                %sumando las condiciones inciales de cada punto.
                t_velcont_y = ((max_height_colunm - (posy_init))-(y_aceled*2))/vy_max;
                if(t_velcont_y>=0)
                    break;
                end
                %vy_max_sc = vy_max_sc-0.0001;
                vy_max = sqrt((max_height_colunm - (posy_init))*ay_max)-0.1;
       end

                %RESET VALUES
                vx_max = 1.0; %[m/s]
                vy_max = vy_max_aux;
                ay_max=1;
                ax_max=1;

                %Armado de las consignas de velocidad.
                %Desde punto 0 hasta punto 1:

                %Tiempo total de trayectoria
                t_total_0 = (time_max_acel_y*2)+t_velcont_y;
                t=0:dt:t_total_0;
                ay0_t = [];   
                %Determinacion de aceleraciones para cada instante de tiempo 
                for u=1:length(t)
                    if(t(u)<=time_max_acel_y)   
                        ay0_t(u)=1;
                    elseif((t(end)-t(u))<=time_max_acel_y)
                        ay0_t(u)=-1;
                    else
                        ay0_t(u)=0;
                    end
                end

                %Integracion acumulativa, obtengo el perfil de velocidad.
                dy0_t = cumtrapz(t,ay0_t);
                %Integracion acumulativa, obtengo el perfil de posicion. Solo para 
                %corroborar, el perfil que usamos es de velocidad.
                y0_t = posy_init + cumtrapz(t,dy0_t);
                %Vector de 0s, porque solo se mueve en y.
                dx0_t = 0*t;

                %ACA SE ARMA LA TRAYECTORIA
                trayectoria_dx = [dx0_t' , t'];
                %Negativo por la convencion de izaje.
                trayectoria_dy = [-dy0_t', t'];
                
        while(true)
            time_max_acel_x =(vx_max)/ax_max;
            %time_max_acel_x =(vx_max)/ax_max;
            x_aceled = (ax_max/2)*(time_max_acel_x^2);
            t_velcont_x = (abs((x_positions(posx_end)-posx_init)) - 2*x_aceled)/(vx_max);
            %t_velcont_x = ((x_positions(posx_end)-posx_init) - 2*x_aceled)/(vx_max);
            if(t_velcont_x>=0)
                break;
            end
            %vx_max=vx_max-0.0001;
            vx_max = sqrt( abs((x_positions(posx_end)-posx_init)) * ax_max ) - 0.1;
        end
        
        t_total_1 = (time_max_acel_x*2)+t_velcont_x;
        
        t=dt:dt:t_total_1;
      
        ax1_t = [];   

        for u=1:length(t)
            if(t(u)<=time_max_acel_x)   
                ax1_t(u)=1;
            elseif((t(end)-t(u))<=time_max_acel_x)
                ax1_t(u)=-1;
            else
                ax1_t(u)=0;
            end
        end


        t = t + t_total_0;
        dx1_t = cumtrapz(t,ax1_t);        
        dy1_t = dx1_t*0;
        trayectoria_dx =[trayectoria_dx; [-dx1_t' , t']];
        trayectoria_dy =[trayectoria_dy; [dy1_t' , t']];
        
        while(true)    
            time_max_acel_y =vy_max/ay_max;
            y_aceled = (ay_max/2)*(time_max_acel_y^2);
            %Aca es donde especifican las distancia desde la posicion en la
            %direccion correspondiente donde estot hasta el siguiente punto.
            %Son todos deplazamientos relativos, despues arma la trayectoria
            %sumando las condiciones inciales de cada punto.
            t_velcont_y = ((y0_t(end)-(estado_barco(posx_end)*hy_cont - boat_under_water + safety_distance))-(y_aceled*2))/vy_max;
            if(t_velcont_y>=0)
            break;
            end
            vy_max=vy_max-0.01;
            end

            %RESET VALUES
            vx_max = 1.0; %[m/s]
            vy_max = vy_max_aux; 
            ay_max=1;
            ax_max=1;

            %Armado de las consignas de velocidad.
            %Desde punto 0 hasta punto 1:

            %Tiempo total de trayectoria
            t_total_end = (time_max_acel_y*2)+t_velcont_y;
            t=dt:dt:t_total_end;
            ayend_t = [];   
            %Determinacion de aceleraciones para cada isntante de tiempo 
            for u=1:length(t)
                if(t(u)<=time_max_acel_y)   
                    ayend_t(u)=1;
                elseif((t(end)-t(u))<=time_max_acel_y)
                    ayend_t(u)=-1;
                else
                    ayend_t(u)=0;
                end
            end

            %Integracion acumulativa, obtengo el perfil de velocidad.
            dyend_t_y = cumtrapz(t,ayend_t);
            %Integracion acumulativa, obtengo el perfil de posicion. Solo para 
            %corroborar, el perfil que usamos es de velocidad.

            %Vector de 0s, porque solo se mueve en y.
            dxend_t_x = 0*t;

        %     Final de la trayectoria
            vxt_end= dxend_t_x';
            %Negativo por la convencion de izaje.
            vyt_end= dyend_t_y';
            
            vyt = trayectoria_dy(:,1);
            vxt = trayectoria_dx(:,1);
            x_end = x_positions(posx_end);
    end
%     
len = length(vxt);
%     x_to_boat=cumtrapz(trayectoria_dx(:,2),trayectoria_dx(:,1))+posx_init;
%     y_to_boat=-cumtrapz(trayectoria_dy(:,2),trayectoria_dy(:,1))+posy_init;
%     y_to_boat=[y_to_boat; y_to_boat(end)-cumtrapz(t,vyt_end)];
%     x_to_boat=[x_to_boat; x_to_boat(end)+cumtrapz(t,vxt_end)];
%     
% % 
%     figure(5)
%         plot(trayectoria_dx(:,2),trayectoria_dx(:,1))
%         hold on
%         plot(trayectoria_dy(:,2),trayectoria_dy(:,1))
%         hold on
% %     
%     plot_scene(estado_barco, x_positions, hy_cont, hx_cont, deltax_cont, ysb, boat_under_water)
% 
%         
end

    

    