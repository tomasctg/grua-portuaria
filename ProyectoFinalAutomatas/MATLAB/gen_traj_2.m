function [trayectoria_dy,trayectoria_dx] = gen_traj( estado_barco,posx_init,posy_init,posx_end)
    %Datos
    dt = 0.01;
    vx_max = 4; 
    vy_max_sc = 3; 
    vy_max_cc = 1;
    ay_max=1;
    ax_max=1;
    boat_wide = 5;
    boat_under_water = 20;
    hy_cont = 2.5;
    hx_cont = 2;
    ysb=12;
    
    %Determino la coordenada en x de cada columna
    x_positions = [hx_cont/2];
    for i=2:boat_wide
        x_positions(i)=(x_positions(i-1) + hx_cont);
    end
    theta = atan(vy_max_sc/vx_max);
    
    
    %Determino columna con mayor cantidad de contairnes
    [max_height_colunm,max_height_colunm_index] = max(estado_barco(1:posx_end));
    
    %Valor altura de columna referenciado al muelle
    max_height_colunm=max_height_colunm*hy_cont - boat_under_water;
    
    %Chequear si las alturas de las columnas estan por encima o por debajo
    %de la viga testera
    if(max_height_colunm>=ysb)
        y_point1=[];
        for u=1:max_height_colunm_index
            %Calculos geometricos para determinar el punto 1
            d = x_positions(max_height_colunm_index) + abs(posx_init);
            h_max=d*tan(theta);
            y_max = estado_barco(u)*hy_cont - boat_under_water;
            %Vector con las alturas de elevacion vertical hasta punto 1,
            %representando cada indice el calculo para cada columna.
            y_point1(u)=y_max-h_max;
            flag=1;
        end
    else
        d = abs(posx_init);
        h_max=d*tan(theta);
        y_point1(1)=ysb-h_max;
        flag=0;
    end

    
    %Trayectorias punto 0 a punto 1: xt e izaje (x0 e y0 respectivamente)
        %Esto despues no se usa, pero lo hice para corroborar la curva;
        %Tomo el punto mas alto de las intersecciones de las rectas con la
        %vertical, almacenados en el vector y_point1.
    %Genero la recta.
    deltay_part0 = max(y_point1) - posy_init;
    delta_t_part0 = deltay_part0/vy_max_sc;
    t_part0=0:dt:(delta_t_part0+0.0001);
    y0 = vy_max_sc*t_part0 + posy_init;
    x0 = t_part0*0 + posx_init;
    
    %ACA ARRANCA LO IMPORTANTE
    %Use las aceleraciones maximas en cada direccion, para calcular los
    %perfiles de velocidad para x e y entre los puntos 0-1; 1-2.  
    
    %Como tengo una aceleracion maxima, lo primero que hago es calcular el
    %tiempo que lleva llegar a la velocidad maxima con aceleracion maxima.
    %Luego con ese tiempo, calculo el desplazamiento en ese periodo de
    %tiempo (amx/2 * t^2). Ese desplazamiento lo multiplico por dos, porque
    %despues hay que desacelerar de la misma forma que se acelero. Asi
    %calculo el tiempo que deberia estar moviendome a velocidad_max
    %constante para que cuando se desacelere y se detenga el movimiento, se
    %cumpla con la distancia requerida. IMPORTANTE, puede ser el caso que
    %la distanacia a recorrer sea tan chica que el movimiento no alcanze su
    %velocidad maxima y tenga que desacelerar. Es decir, el perfil de
    %aceleracion no tendra tiempo muerto y se acortaran los tiempos a los
    %cuales se mantiene una aceleracion_max constante. 
    %Para eso use un while donde chequeo que el tiempo a vel constante no
    %sea negativo, reactualizando la velocidad maxima hasta que ese tiempo
    %sea nulo. En el caso que sea positivo de entrada, se verifica el if y
    %sale directamente.
    while(true)    
    time_max_acel_y =vy_max_sc/ay_max;
    y_aceled = (ay_max/2)*(time_max_acel_y^2);
    %Aca es donde especifican las distancia desde la posicion en la
    %direccion correspondiente donde estot hasta el siguiente punto.
    %Son todos deplazamientos relativos, despues arma la trayectoria
    %sumando las condiciones inciales de cada punto.
    t_velcont_y = (deltay_part0-(y_aceled*2))/vy_max_sc;
    if(t_velcont_y>=0)
        break;
    end
    vy_max_sc=vy_max_sc-0.0001;
    end
    
    %RESET VALUES
    vx_max = 4; %[m/s]
    vy_max_sc = 3; 
    vy_max_cc = 1;
    ay_max=1;
    ax_max=1;
    
    %Armado de las consignas de velocidad.
    %Desde punto 0 hasta punto 1:
    
    %Tiempo total de trayectoria
    t_total_0 = (time_max_acel_y*2)+t_velcont_y;
    t=0:dt:t_total_0;
    ay0_t = [];   
    %Determinacion de aceleraciones para cada isntante de tiempo 
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
    
    
    %Recta desde punto1 hasta punto2(max altura), NO ES IMPORTANTE
    x_part1_abs=[0:dt:(x_positions(max_height_colunm_index)*flag-posx_init)];
    x_part1 = [posx_init:dt:x_positions(max_height_colunm_index)*flag];
    y_part1=tan(theta)*x_part1_abs + max(y_point1);
       
    
    %Trayectorias punto 1 a punto 2: xt e izaje (x0 e y0 respectivamente),
    %NO ES IMPORTANTE
    delta_x_part1 = x_positions(max_height_colunm_index)*flag-posx_init;
    delta_t_part1=delta_x_part1/vx_max; 
    t_part1=dt:dt:(delta_t_part1+dt);
    x1=vx_max*t_part1 + posx_init;
    y1=vy_max_sc*t_part1 + y0(length(y0)); 
    t_part1 = t_part1+ t_part0(length(t_part0)); 
    
    
    %COMIENZA CALCULO DE TRAYECTORIAS DESDE PUNTO 1 a 2.
    %Hago lo mismo que explique mas arriba. Solo que ahora para la
    %direccion en x desde el punto 1 al punto 2.
    
    %LOOP GENERAL PARA LOGRAR SINCRONIA EN EL MOVIMIENTO
    k=1;
    while(true)  
        
        while(true)
        
        time_max_acel_x =(vx_max*k)/ax_max;
        x_aceled = (ax_max/2)*(time_max_acel_x^2);
        t_velcont_x = ((x_positions(posx_end)-posx_init) - 2*x_aceled)/(vx_max*k);
        if(t_velcont_x>=0)
            break;
        end
        vx_max=vx_max-0.0001;
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

        %Importante para tener un aumento monotono temporal.

        t = t + t_total_0;
        dx1_t = cumtrapz(t,ax1_t);
        x1_t = cumtrapz(t,dx1_t) + posx_init;
        trayectoria_dx =[trayectoria_dx_aux; [dx1_t' , t']];

        x_total = (posx_init + cumtrapz(trayectoria_dx(:,2),trayectoria_dx(:,1)))';

        % Desde y0 hasta y1.

        %Distancia al siguiente obstaculo
        d_obs1 = x_positions(max_height_colunm_index);

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
        deltay1 = max_height_colunm - y0_t(end);
        Amax1 = ay_max;

        prof_vel_1_2 = solve_profile_vel(deltay1,deltat1,Amax1);

        y_point3=[];
            for u=max_height_colunm_index:posx_end
                %Calculos geometricos para determinar el punto 1
                d = x_positions(posx_end)-x_positions(u);
                h_max=d*tan(theta);
                y_max = estado_barco(u)*hy_cont - boat_under_water;
                %Vector con las alturas de elevacion vertical hasta punto 1,
                %representando cada indice el calculo para cada columna.
                y_point3(u)=y_max-h_max;
            end

        %Datos para solver de punto 2 a 3
        deltat2 = t_obs2 - t_obs1;
        deltay2 = max_height_colunm - max(y_point3);
        Amax2 = ay_max;

        prof_vel_2_3 = solve_profile_vel(deltay2,deltat2,Amax2);
        
        if( ~(isempty(prof_vel_1_2.ts)) && ~(isempty(prof_vel_2_3.ts)) )
            break;
        end
        k=k-0.01;
        
        t = [];
        trayectoria_dx =[];
        x_total = [];       
    end
    
    t_total_1_y = double(prof_vel_1_2.ta*2 + prof_vel_1_2.ts);
    t=dt:dt:t_total_1_y;
    vy1_t = [];   

    for u=1:length(t)
        if(t(u)<=prof_vel_1_2.ta)   
            vy1_t(u)=(t(u)*double(prof_vel_1_2.vmax))/double(prof_vel_1_2.ta);
        elseif((t(end)-t(u))<=double(prof_vel_1_2.ta))
            vy1_t(u)=double(prof_vel_1_2.vmax)*(1 - (t(u)-(double(prof_vel_1_2.ta+prof_vel_1_2.ts)))/double(prof_vel_1_2.ta));
        else
            vy1_t(u)=double(prof_vel_1_2.vmax);
        end
    end
    
    t=t+trayectoria_dy(end,2);
    trayectoria_dy = [trayectoria_dy ; [-vy1_t', t']];
    
    
    t_total_2_y = double(prof_vel_2_3.ta*2 + prof_vel_2_3.ts);
    t=dt:dt:t_total_2_y;
    vy2_t = [];   
    
    for u=1:length(t)
        if(t(u)<=prof_vel_2_3.ta)   
            vy2_t(u)=(t(u)*double(prof_vel_2_3.vmax))/double(prof_vel_2_3.ta);
        elseif((t(end)-t(u))<=double(prof_vel_2_3.ta))
            vy2_t(u)=double(prof_vel_2_3.vmax)*(1 - (t(u)-(double(prof_vel_2_3.ta+prof_vel_2_3.ts)))/double(prof_vel_2_3.ta));
        else
            vy2_t(u)=double(prof_vel_2_3.vmax);
        end
    end
    
    
    t=t+trayectoria_dy(end,2);
    trayectoria_dy = [trayectoria_dy; [vy2_t', t']];
    y2_t=(-cumtrapz(trayectoria_dy(:,2),trayectoria_dy(:,1))+posy_init);

        %COMO USO EL BLOQUE DE REPETE SEQUENCE, TENGO QUE CONTINUAR 
        %LA TRAYECTORIA UN TIEMPO LARGO, PARA QUE NO SE VUELVA A REPETIR 
        %LA CONSIGNA EN LA SIMULACION.
        t_end_y=(dt+trayectoria_dy(end,2):dt:40);
        t_end_x=(dt+trayectoria_dx(end,2):dt:40);
        dx_end = 0*t_end_x;
        dy_end = 0*t_end_y;
        
        %RESULTADO FINAL. 
        trayectoria_dy = [trayectoria_dy; [dy_end',t_end_y']];
        trayectoria_dx = [trayectoria_dx; [dx_end',t_end_x']];
 
        
    while(true)    
    time_max_acel_y =vy_max_sc/ay_max;
    y_aceled = (ay_max/2)*(time_max_acel_y^2);
    %Aca es donde especifican las distancia desde la posicion en la
    %direccion correspondiente donde estot hasta el siguiente punto.
    %Son todos deplazamientos relativos, despues arma la trayectoria
    %sumando las condiciones inciales de cada punto.
    t_velcont_y = ((y2_t(end)-(estado_barco(posx_end)*hy_cont - boat_under_water))-(y_aceled*2))/vy_max_sc;
    if(t_velcont_y>=0)
        break;
    end
    vy_max_sc=vy_max_sc-0.0001;
    end
    
    %RESET VALUES
    vx_max = 4; %[m/s]
    vy_max_sc = 3; 
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
    
    t=t+trayectoria_dy(end,2);
    
    %ACA SE ARMA LA TRAYECTORIA
    trayectoria_dx= [trayectoria_dx; [dxend_t_x' , t']];
    %Negativo por la convencion de izaje.
    trayectoria_dy= [trayectoria_dy; [dyend_t_y' , t']];
        
    
    t_end_y=(dt+trayectoria_dy(end,2):dt:150);
    t_end_x=(dt+trayectoria_dx(end,2):dt:150);
    dx_end = 0*t_end_x;
    dy_end = 0*t_end_y;
        
        %RESULTADO FINAL. 
    trayectoria_dy = [trayectoria_dy; [dy_end',t_end_y']];
    trayectoria_dx = [trayectoria_dx; [dx_end',t_end_x']];
 
 
        
end

