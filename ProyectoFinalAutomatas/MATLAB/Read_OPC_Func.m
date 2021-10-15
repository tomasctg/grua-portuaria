function x = Read_OPC_Func(y)
%Variables
persistent init_Server;
persistent init_Nodes;
persistent ua_Client;

%Nodes
    %Level 0
%         %Writeable Variables
        persistent xt;
        persistent yl;
        persistent vyt;		
        persistent switch_energy;
% 		persistent balancer_switch;
%         persistent twistlock;
% 		persistent switch_mode;	
        persistent Stable_FW_flag;
% 
%     %Level 1
        %Writeable Variables
        persistent  Fw;
        persistent theta_amp;
        persistent xend;
  
        %Readable Variables
        persistent balancer;
        persistent flag_add_random_mass;
        persistent rand_mass;
        persistent flag_semiauto;
        persistent limit_going_down;
        persistent limit_vxt_down;
        persistent mode_balance;
        persistent reset_calc_mass;
        persistent flag_end_traj;
        persistent flag_send_traj;
        persistent reset_going;
        persistent reset_down;
        persistent Load_state;
        persistent flag_gen_traj_boat;
		persistent flag_gen_traj_dock;
		


%initializate variables
if(isempty(init_Server))
    init_Server = 0;
    init_Nodes = 0;
end

if init_Server == 0
    init_Server = 1;
        server = opcuaserverinfo('localhost');
        ua_Client = opcua(server);
        %connect(ua_Client,'tom','tom');
        connect(ua_Client, 'gabrielq', 'incorrecta1');
end

if ua_Client.isConnected && init_Nodes == 0
    init_Nodes =1;
    DB_Node = findNodeByName(ua_Client.Namespace,'GVL0','-once');

    %READ
    balancer = findNodeByName(DB_Node,'balancer','-once');
    flag_add_random_mass = findNodeByName(DB_Node,'flag_add_random_mass','-once');
    rand_mass = findNodeByName(DB_Node,'rand_mass','-once');
    flag_semiauto = findNodeByName(DB_Node,'flag_semiauto','-once');
    limit_going_down = findNodeByName(DB_Node,'limit_going_down','-once');
    limit_vxt_down = findNodeByName(DB_Node,'limit_vxt_down','-once');
    mode_balance = findNodeByName(DB_Node,'mode_balance','-once');
    reset_calc_mass = findNodeByName(DB_Node,'reset_calc_mass','-once');
    flag_end_traj = findNodeByName(DB_Node,'flag_end_traj','-once');
    flag_send_traj = findNodeByName(DB_Node,'flag_send_traj','-once');
    reset_going = findNodeByName(DB_Node,'reset_going','-once');
    reset_down = findNodeByName(DB_Node,'reset_down','-once');
    Load_state = findNodeByName(DB_Node,'Load_state','-once');    
    flag_gen_traj_boat = findNodeByName(DB_Node,'flag_gen_traj_boat','-once');
    flag_gen_traj_dock = findNodeByName(DB_Node,'flag_gen_traj_dock','-once');   
    %WRITE
    xt = findNodeByName(DB_Node,'xt','-once');
    yl = findNodeByName(DB_Node,'yl','-once');
    vyt = findNodeByName(DB_Node,'vyt','-once');    
    Fw = findNodeByName(DB_Node,'Fw','-once');
    theta_amp = findNodeByName(DB_Node,'theta_amp','-once'); 
    switch_energy = findNodeByName(DB_Node,'switch_energy','-once');
%     balancer_switch = findNodeByName(DB_Node,'balancer_switch','-once');    
%     twistlock = findNodeByName(DB_Node,'twistlock','-once');
%     switch_mode = findNodeByName(DB_Node,'switch_mode','-once'); 
    Stable_FW_flag = findNodeByName(DB_Node,'Stable_FW_flag','-once');
    xend = findNodeByName(DB_Node,'xend','-once');
end
if ua_Client.isConnected == 1 && init_Nodes==1
    
    [Balancer, ~,~] = readValue(ua_Client,balancer);
     [Flag_add_random_mass, ~,~] = readValue(ua_Client,flag_add_random_mass);
      [Rand_mass, ~,~] = readValue(ua_Client,rand_mass);
       [Flag_semiauto, ~,~] = readValue(ua_Client,flag_semiauto);
        [Limit_going_down, ~,~] = readValue(ua_Client,limit_going_down);
         [Limit_vxt_down, ~,~] = readValue(ua_Client,limit_vxt_down);
          [Mode_balance, ~,~] = readValue(ua_Client,mode_balance);
           [Reset_calc_mass, ~,~] = readValue(ua_Client,reset_calc_mass);
            [Flag_end_traj, ~,~] = readValue(ua_Client,flag_end_traj);
             [Flag_send_traj, ~,~] = readValue(ua_Client,flag_send_traj);
              [Reset_going, ~,~] = readValue(ua_Client,reset_going);
               [Reset_down, ~,~] = readValue(ua_Client,reset_down);
                [load_state, ~,~] = readValue(ua_Client,Load_state);
                 [Flag_gen_traj_boat, ~,~] = readValue(ua_Client,flag_gen_traj_boat);
                  [Flag_gen_traj_dock, ~,~] = readValue(ua_Client,flag_gen_traj_dock);
                   [Switch_energy, ~,~] = readValue(ua_Client,switch_energy);
    
    writeValue(ua_Client,xt,y(1));
    writeValue(ua_Client,yl,y(2));
    writeValue(ua_Client,vyt,y(3));
    writeValue(ua_Client,Fw,y(4));
    writeValue(ua_Client,theta_amp,y(5));
%     writeValue(ua_Client,switch_energy,y(6));
%     writeValue(ua_Client,balancer_switch,y(7));
%     writeValue(ua_Client,twistlock,y(8));
%     writeValue(ua_Client,switch_mode,y(5));
    writeValue(ua_Client,Stable_FW_flag,y(6));
    writeValue(ua_Client,xend,y(7));
end
x = double([Balancer Flag_add_random_mass Rand_mass Flag_semiauto Limit_going_down Limit_vxt_down Mode_balance Reset_calc_mass Flag_end_traj Flag_send_traj Reset_going Reset_down load_state Flag_gen_traj_boat Flag_gen_traj_dock Switch_energy]); 
% x = double(Balancer);
end

