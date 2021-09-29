function [vyt,vxt,x_end,vxt_end,vyt_end,len,len2,estado_barco2] = gen_traj_to_boat(estado_barco,posx_init,posy_init,posx_end,twistlocks, ml)
    tic
    %Datos
    dt = 0.5e-3;
    
    POT = 97500; %=32500*3=65000*1.5
    if(twistlocks)
        if ml <= 32500
            vy_max_aux = 3;
        elseif ml <= 65000
            vy_max_aux = POT/ml;
        end
    else
        vy_max_aux = 3;
    end
    
    vy_max = vy_max_aux;
    vx_max = 4;
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
    for i=2:boat_wide
        x_positions(i)=(x_positions(i-1) + hx_cont + deltax_cont);
    end
    theta = atan(vy_max/vx_max);
    
    
    %Determino columna con mayor cantidad de containers
    [max_height_colunm,max_height_colunm_index] = max(estado_barco(1:posx_end));
    
    %Valor altura de columna referenciado al muelle
    max_height_colunm=max_height_colunm*hy_cont - boat_under_water + safety_distance;
    
    %Chequear si las alturas de las columnas estan por encima o por debajo
    %de la viga testera
    if(max_height_colunm>=ysb+safety_distance)
        y_point1=[];
        for u=1:max_height_colunm_index
            %Calculos geometricos para determinar el punto 1
            d = x_positions(u) + abs(posx_init);
            h_max=d*tan(theta);
            y_max = estado_barco(u)*hy_cont - boat_under_water;
            %Vector con las alturas de elevacion vertical hasta punto 1,
            %representando cada indice el calculo para cada columna.
            y_point1(u)=y_max-h_max+safety_distance;
            flag=1;
        end
    else
        d = abs(posx_init);
        h_max=d*tan(theta);
        y_point1(1)=ysb-h_max+safety_distance;
        flag=0;
    end

    
    %Trayectorias punto 0 a punto 1: xt e izaje (x0 e y0 respectivamente)
        %Esto despues no se usa, pero lo hice para corroborar la curva;
        %Tomo el punto mas alto de las intersecciones de las rectas con la
        %vertical, almacenados en el vector y_point1.
    %Genero la recta.
    deltay_part0 = max(y_point1) - posy_init;
    delta_t_part0 = deltay_part0/vy_max;
    t_part0=0:dt:(delta_t_part0);
    y0 = vy_max*t_part0 + posy_init;
    x0 = t_part0*0 + posx_init;
    
    %ACA ARRANCA LO IMPORTANTE
    %Use las aceleraciones maximas en cada direccion, para calcular los
    %perfiles de velocidad para x e y entre los puntos 0-1; 1-2.  

    if(deltay_part0>0)
    
        while(true)    
        time_max_acel_y =vy_max/ay_max;
        y_aceled = (ay_max/2)*(time_max_acel_y^2);
        %Aca es donde especifican las distancia desde la posicion en la
        %direccion correspondiente donde estoy hasta el siguiente punto.
        %Son todos deplazamientos relativos, despues arma la trayectoria
        %sumando las condiciones inciales de cada punto.
        t_velcont_y = (deltay_part0-(y_aceled*2))/vy_max;
        if(t_velcont_y>=0)
            break;
        end
        %vy_max_sc = vy_max_sc-0.0001;
        vy_max = sqrt(deltay_part0*ay_max)-0.1;
        end

        %RESET VALUES
        vx_max = 4; %[m/s]
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
        trayectoria_dx_aux = trayectoria_dx;
        %Negativo por la convencion de izaje.
        trayectoria_dy = [-dy0_t', t'];
    else
        trayectoria_dx_aux= [];
        trayectoria_dy=[];
        trayectoria_dx = [];
        t_total_0=0;
        y0_t=posy_init;
    end
    

    
    %COMIENZA CALCULO DE TRAYECTORIAS DESDE PUNTO 1 a 2.
    %Hago lo mismo que explique mas arriba. Solo que ahora para la
    %direccion en x desde el punto 1 al punto 2.
    
    %LOOP GENERAL PARA LOGRAR SINCRONIA EN EL MOVIMIENTO
  
    k=1;
    while(true)  
        while(true)
        
        time_max_acel_x =(vx_max*k)/ax_max;
        %time_max_acel_x =(vx_max)/ax_max;
        x_aceled = (ax_max/2)*(time_max_acel_x^2);
        t_velcont_x = ((x_positions(posx_end)-posx_init) - 2*x_aceled)/(vx_max*k);
        %t_velcont_x = ((x_positions(posx_end)-posx_init) - 2*x_aceled)/(vx_max);
        if(t_velcont_x>=0)
            break;
        end
        %vx_max=vx_max-0.0001;
        vx_max = sqrt( ( x_positions(posx_end)-posx_init ) * ax_max ) / k - 0.1;
        end

        
        t_total_1 = (time_max_acel_x*2)+t_velcont_x;
        if(flag==0)
            t=0:dt:t_total_1;
            t_total_0=0;
        else    
            t=dt:dt:t_total_1;
        end
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
%         x1_t = cumtrapz(t,dx1_t) + posx_init;
        
        if(flag==0)
            trayectoria_dx =[dx1_t' , t'];
        else
            if(~isempty(trayectoria_dx_aux))
            trayectoria_dx =[trayectoria_dx_aux; [dx1_t' , t']];
            else
            trayectoria_dx=[dx1_t' , t'];
            end
        end
        
        x_total = (posx_init + cumtrapz(trayectoria_dx(:,2),trayectoria_dx(:,1)))';

        % Desde y0 hasta y1.

        %Distancia al siguiente obstaculo
        if(flag==0)
            d_obs1=0;
            y0_t=posy_init;
        else
            d_obs1 = x_positions(max_height_colunm_index);
        end
        %Obtencion de tiempo en el cual el carro se encuentra a la distancia
        %d_obs1.

        for u=1:length(x_total)
            if((x_total(u)-d_obs1) <= 0.005)
                t_obs1 = trayectoria_dx(u,2);
            end
            if((x_total(u)-x_positions(posx_end)) <= 0.005)
                t_obs2 = trayectoria_dx(u,2);
            end
        end

       %Datos para entrar al solver del perfil para el izaje: Deltat = t_obs1l Deltay = (y1-y0) Amax
        
        deltat1 = t_obs1 - t_total_0;
        if(flag==0)
            deltay1 = safety_distance + ysb - y0_t(end);
        else
            deltay1 = max_height_colunm - y0_t(end);
        end
   
        Amax1 = ay_max;

        prof_vel_1_2 = solve_vel_prof(deltay1,deltat1,Amax1);

        % Check if the end position is or not the highest column
        if(posx_end ~= max_height_colunm_index || (posx_end == max_height_colunm_index && flag==0))
           if(flag==1)
                y_point3=[];
                for u=max_height_colunm_index:posx_end
                    %Calculos geometricos para determinar el punto 1
                    d = x_positions(posx_end)-x_positions(u);
                    h_max=d*tan(theta);
                    y_max = estado_barco(u)*hy_cont - boat_under_water;
                    %Vector con las alturas de elevacion vertical hasta punto 1,
                    %representando cada indice el calculo para cada columna.
                    y_point3(u)=y_max-h_max+safety_distance;
                end
            else
                    d = x_positions(posx_end);
                    h_max=d*tan(theta);
                    y_point3=ysb-h_max+safety_distance;
            end

            %Datos para solver de punto 2 a 3
            deltat2 = t_obs2 - t_obs1;

            if(flag==0)
                deltay2 = ysb - max(y_point3);
            else
                 deltay2 = max_height_colunm - max(y_point3);
            end
            deltay2 = deltay2 + safety_distance;

            Amax2 = ay_max;

            prof_vel_2_3 = solve_vel_prof(deltay2,deltat2,Amax2);

%             if( ~(isempty(prof_vel_1_2.ts)) && ~(isempty(prof_vel_2_3.ts)) )
            if(prof_vel_1_2(1) <= vy_max_aux && prof_vel_2_3(1) <= vy_max_aux)
                break;
            end
            k=k-0.1;
        else
            if(~(isempty(prof_vel_1_2.ts)))
                break;
            end
            k=k-0.1;
        end
        t = [];
        trayectoria_dx =[];
        x_total = []; 
        
    end
 
   
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    
    
    ta1_2=double(prof_vel_1_2(3));
    ts1_2= double(prof_vel_1_2(2));
    vmax1_2=double(prof_vel_1_2(1));
    
    t_total_1_y = ta1_2*2 +ts1_2;
    if(flag==1)
       t=dt:dt:t_total_1_y;
    else
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
        
    if(flag==1)
        if(~isempty(trayectoria_dy))
        t=t+trayectoria_dy(end,2);
        trayectoria_dy = [trayectoria_dy ; [-vy1_t', t']];
        else
        trayectoria_dy = [trayectoria_dy ; [-vy1_t', t']];
        end
    else
    trayectoria_dy = [-vy1_t', t'];
    end
    

    if(posx_end ~= max_height_colunm_index || (posx_end == max_height_colunm_index && flag==0))
    
    ta2_3=double(prof_vel_2_3(3));
    ts2_3= double(prof_vel_2_3(2));
    vmax2_3=double(prof_vel_2_3(3));
    
    
    t_total_2_y = ta2_3*2 + ts2_3;
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
    
    end
    
    vyt = trayectoria_dy(:,1);
    vxt = trayectoria_dx(:,1);
    x_end = x_positions(posx_end);
    y2_t=(-cumtrapz(trayectoria_dy(:,2),trayectoria_dy(:,1))+posy_init);


    if(posx_end ~= max_height_colunm_index || (posx_end == max_height_colunm_index && flag==0))
        while(true)    
        time_max_acel_y =vy_max/ay_max;
        y_aceled = (ay_max/2)*(time_max_acel_y^2);
        %Aca es donde especifican las distancia desde la posicion en la
        %direccion correspondiente donde estot hasta el siguiente punto.
        %Son todos deplazamientos relativos, despues arma la trayectoria
        %sumando las condiciones inciales de cada punto.
        t_velcont_y = ((y2_t(end)-(estado_barco(posx_end)*hy_cont - boat_under_water + safety_distance))-(y_aceled*2))/vy_max;
        if(t_velcont_y>=0)
        break;
        end
        vy_max=vy_max-0.01;
        end
    
        %RESET VALUES
        vx_max = 4; %[m/s]
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
    else
        vxt_end = 0;
        vyt_end = 0;
    end
    
    len = length(vxt);
    len2 = length(vyt_end);
    
%     for i=1:boat_wide
%         estado_barco2(i) = estado_barco(i);
%     end
    estado_barco2 = estado_barco;
    if (twistlocks)
        estado_barco2(posx_end) = estado_barco(posx_end) + 1;
    end
    estado_barco2 = estado_barco2';
    x_to_boat=cumtrapz(trayectoria_dx(:,2),trayectoria_dx(:,1))+posx_init;
    y_to_boat=-cumtrapz(trayectoria_dy(:,2),trayectoria_dy(:,1))+posy_init;

    
    figure(1)
    plot(x_to_boat,y_to_boat(1:length(x_to_boat),1))
    hold on
    
%     plot(0, ysb, 'o', 'color', 'r')
%     plot(x_positions, estado_barco*hy_cont - boat_under_water, 'o', 'color', 'r')

    figure(1)
    plot_scene(estado_barco2, x_positions, hy_cont, hx_cont, deltax_cont, ysb, boat_under_water)
    
    toc
end


