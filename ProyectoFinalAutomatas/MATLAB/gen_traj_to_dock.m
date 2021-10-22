function [vyt,vxt,x_end,vxt_end,vyt_end,len_going,len_down,estado_barco2] = gen_traj_to_dock(estado_barco,posx_init,posy_init,posx_end,twistlocks,ml)   
    tic
    %Datos
    dt = 0.5e-3;
    
%% Calculo de potencia para determinar velocidad
    POT = 97500; %=32500*3=65000*3.5
    if(twistlocks)
        if ml <= 32501
            vy_max_aux = 3;
        elseif ml <= 65000
            vy_max_aux = POT/ml;
        end
    else
        vy_max_aux = 3;
    end
    
    vy_max = vy_max_aux;
    vx_max = 4.0;
    ay_max=1;
    ax_max=1;

%% Datos generales grua     
    boat_wide = 6;
    dock_wide = 5;
    deltax_cont = 0.2;    
    boat_under_water = 10;
    hy_cont = 2.5;
    hx_cont = 2.44;
    ysb=12;    
    safety_distance=5;
    
%% Coordenada en x de cada columna de contenedores respecto a muelle     
    x_positions = [hx_cont/2 + deltax_cont];
    posx_init_index = 1;
    
    for i=2:boat_wide
        x_positions(i)=(x_positions(i-1) + hx_cont + deltax_cont);
        if(abs(posx_init - x_positions(i)) <= 0.1)
            posx_init_index = i;
        end
    end
    posx_init = posx_init_index;
    
%% Determino la coordenada en x de posiciones en muelle (solo se termino trabajando con una posicion)
    x_positions_dock = -20;
    for i=2:dock_wide
        x_positions_dock(i)=(x_positions_dock(i-1) + hx_cont);
    end
    
    
%% Determino columna con mayor cantidad de contairnes
    
    theta = atan(vy_max/vx_max);
    
    % Se recorren las columnas del barco de izquierza a derecha hasta la
    % posicion incial
    
    [max_height_colunm,max_height_colunm_index] = max(estado_barco(1:posx_init));
    
    %Valor altura de columna referenciado al muelle mas altura de seguridad
    max_height_colunm = max_height_colunm*hy_cont - boat_under_water + safety_distance;
    
    %Chequear si las alturas de las columnas estan por encima o por debajo
    %de la viga testera
    if(max_height_colunm>=ysb+safety_distance)
        y_point1=[];
        for u=max_height_colunm_index:posx_init
            %Calculos geometricos para determinar el punto 1
            d = x_positions(posx_init) - x_positions(u);
            h_max=d*tan(theta);
            y_max = estado_barco(u)*hy_cont - boat_under_water;
            %Vector con las alturas de elevacion vertical hasta punto 1,
            %representando cada indice el calculo para cada columna.
            y_point1(u)=y_max-h_max+safety_distance;
            flag=1;
        end
    else
        d = x_positions(posx_init);
        h_max=d*tan(theta);
        y_point1(1)=(ysb+safety_distance)- h_max;
        flag=0;
    end

    
%% Solucion de primer tramo de elevacion (0 - 1) [En caso de ser necesario]     
    
    %Delta de elevacion
    deltay_part0 = max(y_point1) - posy_init;
    
    %Si la distancia es positiva y mayor que 0 computo la solucion
    if(deltay_part0>0)
        flag_elevation_init = 1;
        %Solver: Se parte del caso mas rapido, maxima aceleracion y maxima velocidad    
        while(true)    

            % Tiempo de aceleracion para llegar a la maxima velocidad      
            time_max_acel_y =vy_max/ay_max;

            % Distancia recorrida en el tiempo de aceleracion calculado
            y_aceled = (ay_max/2)*(time_max_acel_y^2);

            %Aca es donde especifican las distancia desde la posicion en la
            %direccion correspondiente donde estoy hasta el siguiente punto.
            %Son todos deplazamientos relativos, despues arma la trayectoria
            %sumando las condiciones inciales de cada punto.

            %Tiempo con aceleracion nula y velocidad constante maxima
            t_velcont_y = (deltay_part0-(y_aceled*2))/vy_max;

            %Si este tiempo es negativo significa que el perfil no tiene
            %solucion cinematica. 
            if(t_velcont_y>=0)
                break;
            end
            %vy_max_sc = vy_max_sc-0.0001;
            vy_max = sqrt(deltay_part0*ay_max)-0.01;

            %ITERAR DISMINUYENDO LA VELOCIDAD HASTA ENCONTRAR EL PERFIL MAS
            %RAPIDO Y CONSISTENTE
        end

        %RESET VALUES
        vx_max = 4.0; %[m/s]
        vy_max = vy_max_aux;
        ay_max=1;
        ax_max=1;

        %Tiempo total de trayectoria (tiempo acelerando + tiempo a vel
        %constante)
        t_total_0 = (time_max_acel_y*2)+t_velcont_y;

        %Vector de tiempo discretizado en dt
        t=0:dt:t_total_0;

        %Perfil de aceleracion
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
        %corroborar, el perfil que usamos es de velocidad. [PONER DONDE SE
        %VUELVE A USAR] 
        y0_t = posy_init + cumtrapz(t,dy0_t);

        %Perfil de velocidad en direccion de x.
        dx0_t = 0*t;

        %Matriz nx2 [velocidad' , tiempo']: Perfil de velocidad en X
        trayectoria_dx = [dx0_t' , t'];

        %Copia para iterar
        trayectoria_dx_aux = trayectoria_dx;

        %Matriz nx2 [velocidad' , tiempo']: Perfil de velocidad en Y
        trayectoria_dy = [-dy0_t', t'];

    else
        %Inicializacion de matrices
        flag_elevation_init = 0;
        trayectoria_dx_aux= [];
        trayectoria_dy=[];
        trayectoria_dx = [];
        t_total_0=0;
        y0_t=posy_init;
    end
    
%% Solucion de tramo de translacion X (1 - 2)      
    tic
    k=1;
    %Solver: Dos bucles while anidados
    while(true)  
        %% Solver VXT
         if(k<0)
            break
        end
        
        while(true)
            %Dismunuyo velocidad exponencialmente
            time_max_acel_x =(vx_max*k)/ax_max;
          
            %time_max_acel_x =(vx_max)/ax_max;
          
            x_aceled = (ax_max/2)*(time_max_acel_x^2);
            
            t_velcont_x = (abs(x_positions(posx_init)-x_positions_dock(posx_end)) - (2*x_aceled))/(vx_max*k);
            %t_velcont_x = ((x_positions(posx_end)-posx_init) - 2*x_aceled)/(vx_max);
            if(t_velcont_x>=0)
                break;
            end
            vx_max=vx_max-0.0001;
            %vx_max = sqrt( (x_positions(posx_init)-x_positions_dock(posx_end)) * ax_max ) / k - 0.01;
        end

        %Tiempo de segunda trayectoria: Son todas relativas 
        t_total_1 = (time_max_acel_x*2)+t_velcont_x;
        
        %Detectar si ya hubo una trayectoria de elevacion
        if(flag_elevation_init==0)
            t=0:dt:t_total_1;
        elseif(flag_elevation_init==1)    
            t=dt:dt:t_total_1;
        end
        
        %Perfil aceleracion en direccion X
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

        %Importante para tener un aumento monotono temporal.
        t = t + t_total_0;
        dx1_t = cumtrapz(t,ax1_t);
        
        if(flag_elevation_init==0)
            trayectoria_dx =[-dx1_t' , t'];
        elseif(flag_elevation_init==1)
%             if(~isempty(trayectoria_dx_aux))
                trayectoria_dx =[trayectoria_dx_aux; [-dx1_t' , t']];
%             else
%                 trayectoria_dx=[-dx1_t' , t'];
%             end
        end
        
        x_total = (x_positions(posx_init) + cumtrapz(trayectoria_dx(:,2),trayectoria_dx(:,1)))';
       
        %% SOLVER VYT
        
        %Distancia en direccion de X al siguiente obstaculo
        if(flag==0)
            d_obs1=0;
        elseif(flag==1)
            d_obs1 = x_positions(max_height_colunm_index);
        end
        
        %Obtencion de tiempo en el cual el carro se encuentra a la distancia
        %d_obs1.
        for u=1:length(x_total)
            if(abs(x_total(u)-d_obs1) <= 0.01)
                t_obs1 = trayectoria_dx(u,2);
            end
%             if((abs(x_total(u)-x_positions_dock(posx_end))) <= 0.01)
%                 t_obs2 = trayectoria_dx(u,2);
%             end
        end
        t_obs2 = trayectoria_dx(end,2);

        %Datos para entrar al solver del perfil para el izaje: Deltat = t_obs1l Deltay = (y1-y0) Amax        
        deltat1 = t_obs1 - t_total_0;
        if(flag==0)
            deltay1 = (ysb+safety_distance) - y0_t(end);
        elseif(flag==1)
            deltay1 = max_height_colunm - y0_t(end);
        end

        Amax1 = ay_max;

        %Sistemas de ecuaciones no lineales
        prof_vel_1_2 = solve_profile_vel(deltay1,deltat1,Amax1);
        
        %% YA NOS ENCONTRAMOS EN EL SEGUNDO PUNTO
        
        %Si estoy sobrepasando una columna, que es mas alta que la vara
        %testera. 
        if(flag==1)
            y_point3=[];
            for u=1:max_height_colunm_index
                d = x_positions(u)-x_positions_dock(posx_end);
                h_max=d*tan(theta);
                y_max = estado_barco(u)*hy_cont - boat_under_water;
                y_point3(u)=y_max-h_max+safety_distance;
            end
        elseif(flag==0)
                d =-x_positions_dock(posx_end);
                h_max=d*tan(theta);
                y_point3=ysb-h_max+safety_distance;
        end
            
        %Datos para solver de punto 2 a 3
        deltat2 = t_obs2 - t_obs1;
        if(flag==0)
            if(max(y_point3) < (hy_cont+safety_distance))
            deltay2 = ysb +safety_distance - max(y_point3) - safety_distance;
            flag_not_going_down = 1;
            else
             deltay2 = ysb +safety_distance - max(y_point3);
            end
        else
            if(max(y_point3) < (hy_cont+safety_distance))
            deltay2 = max_height_colunm - max(y_point3) - safety_distance;
            flag_not_going_down = 1;
            else
             deltay2 = max_height_colunm - max(y_point3);
             flag_not_going_down = 0;
            end
        end
       
        Amax2 = ay_max;

        prof_vel_2_3 = solve_profile_vel(deltay2,deltat2,Amax2);
        
        if( ~(isempty(prof_vel_1_2.ts)) && ~(isempty(prof_vel_2_3.ts)) )
            break;
        end
        k=k-0.1;
        t = [];
        trayectoria_dx =[];
        x_total = [];       
    end
        
        
    
    ta1_2=double(prof_vel_1_2.ta);
    ts1_2= double(prof_vel_1_2.ts);
    vmax1_2=double(prof_vel_1_2.vmax);
    
   
    t_total_1_y = (ta1_2*2 + ts1_2);
    
    if(flag_elevation_init==1)
       t=dt:dt:t_total_1_y;
    elseif(flag_elevation_init==0)
       t=0:dt:t_total_1_y; 
    end

    vy1_t = [];   
    for u=1:length(t)
        if(t(u)<=ta1_2)   
            vy1_t(u)=(t(u)*(vmax1_2))/(ta1_2);
        elseif((t(end)-t(u))<=(ta1_2))
            vy1_t(u)=(vmax1_2)*(1 - (t(u)-((ta1_2+ts1_2)))/(ta1_2));
        else
            vy1_t(u)=(vmax1_2);
        end
    end
    
    
    if(flag_elevation_init==1)
        if(~isempty(trayectoria_dy))
        t=t+trayectoria_dy(end,2);
        trayectoria_dy = [trayectoria_dy ; [-vy1_t', t']];
        else
        trayectoria_dy = [trayectoria_dy ; [-vy1_t', t']];
        end
    elseif(flag_elevation_init==0)
        trayectoria_dy = [-vy1_t', t'];
    end
    
    ta2_3=double(prof_vel_2_3.ta);
    ts2_3= double(prof_vel_2_3.ts);
    vmax2_3=double(prof_vel_2_3.vmax);
    
    
    t_total_2_y = (ta2_3*2 + ts2_3);
    t=dt:dt:t_total_2_y;
    vy2_t = [];   
    
    for u=1:length(t)
        if(t(u)<=ta2_3)   
            vy2_t(u)=(t(u)*(vmax2_3))/(ta2_3);
        elseif((t(end)-t(u))<=(ta2_3))
            vy2_t(u)=(vmax2_3)*(1 - (t(u)-((ta2_3+ts2_3)))/(ta2_3));
        else
            vy2_t(u)=(vmax2_3);
        end
    end
    
    
    t=t+trayectoria_dy(end,2);
    trayectoria_dy = [trayectoria_dy; [vy2_t', t']];
    vyt = trayectoria_dy(:,1);
    vxt = trayectoria_dx(:,1);
    x_end = x_positions_dock(posx_end);
    y2_t=(-cumtrapz(trayectoria_dy(:,2),trayectoria_dy(:,1))+posy_init);
 
        
    if(~flag_not_going_down)
        while(true)    
    time_max_acel_y =vy_max/ay_max;
    y_aceled = (ay_max/2)*(time_max_acel_y^2);
    %Aca es donde especifican las distancia desde la posicion en la
    %direccion correspondiente donde estot hasta el siguiente punto.
    %Son todos deplazamientos relativos, despues arma la trayectoria
    %sumando las condiciones inciales de cada punto.
    t_velcont_y = ((y2_t(end)-hy_cont-safety_distance)-(y_aceled*2))/vy_max;
    if(t_velcont_y>=0)
        break;
    end
    vy_max=vy_max-0.01;
        end
        %RESET VALUES
        vx_max = 4.0; %[m/s]
        vy_max = vy_max_aux; 
        vy_max_cc = 1;
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

        %Final de la trayectoria
        vxt_end= dxend_t_x';
        %Negativo por la convencion de izaje.
        vyt_end= dyend_t_y';
    else
        vxt_end = zeros(1,1000)';
        vyt_end = zeros(1,1000)';
    end
    toc

    
    len_going = length(vxt);
    len_down = length(vxt_end);
    estado_barco2 = estado_barco;
    
    if (twistlocks)
        estado_barco2(posx_init) = estado_barco(posx_init) - 1;
    end
    estado_barco2 = estado_barco2';

%     x_to_boat=cumtrapz(trayectoria_dx(:,2),trayectoria_dx(:,1))+x_positions(posx_init);
%     y_to_boat=-cumtrapz(trayectoria_dy(:,2),trayectoria_dy(:,1))+posy_init;
% 
%     figure(1) 
%     plot(trayectoria_dx(:,2),x_to_boat)
%     hold on
%     plot(trayectoria_dy(:,2),y_to_boat)
% %     
%     figure(2)
%     plot(x_to_boat,y_to_boat)
%     hold on
% %   
%     try
%     figure(2)
%     plot(x_to_boat,y_to_boat(length(x_to_boat),1))
%     hold on
%     catch
%     figure(2)
%     plot(x_to_boat(length(y_to_boat),1),y_to_boat)
%     hold on
%     end
    
%     figure(3)
%     plot(trayectoria_dy(:,2),trayectoria_dy(:,1))
% %     
% %     plot(0, ysb, 'o', 'color', 'r')
% %     plot(x_positions, estado_barco*hy_cont - boat_under_water, 'o', 'color', 'r')
% 

plot_scene(estado_barco2, x_positions, hy_cont, hx_cont, deltax_cont, ysb, boat_under_water)

        
end
