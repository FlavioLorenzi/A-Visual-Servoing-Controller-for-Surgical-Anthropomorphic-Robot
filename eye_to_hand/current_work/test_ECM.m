%%
%   INIT STUFF

%                               Sorta di "mode 0" per l'ECM dove data la
%                               home pose deve convergere con la INV_KIN
                                
%%
cd(fileparts(mfilename('fullpath')));
clear;
close all;
clc;

pause(3);
%%
% CONNECTION TO VREP
%%

[ID,vrep] = utilsECM.init_connection();

% COLLECTING HANDLES

% vision sensor
[~, h_VS] =vrep.simxGetObjectHandle(ID, 'Vision_sensor_ECM', vrep.simx_opmode_blocking);

% as end effector Consider the entire final structure
[~, h_EE] =vrep.simxGetObjectHandle(ID, 'L4_visual_ECM', vrep.simx_opmode_blocking);   


[~, h_PSM]=vrep.simxGetObjectHandle(ID, 'EE', vrep.simx_opmode_blocking);  %Utile per metterlo in contatto con PSM
[~, h_RCM]=vrep.simxGetObjectHandle(ID, 'RCM_PSM1', vrep.simx_opmode_blocking);

% reference for direct kin
%joints (R R P R)
[~, h_j1] = vrep.simxGetObjectHandle(ID,'J1_ECM',vrep.simx_opmode_blocking);
[~, h_j2] = vrep.simxGetObjectHandle(ID,'J2_ECM',vrep.simx_opmode_blocking);
[~, h_j3] = vrep.simxGetObjectHandle(ID,'J3_ECM',vrep.simx_opmode_blocking);
[~, h_j4] = vrep.simxGetObjectHandle(ID,'J4_ECM',vrep.simx_opmode_blocking);


% collection of all joint handles
h_joints = [h_j1; h_j2; h_j3; h_j4];

sync = utilsECM.syncronize(ID, vrep, h_joints, h_VS, h_EE, h_PSM);

if sync
    fprintf(1,'Sycronization: OK... \n');
    pause(1);
end

% end effector home pose wrt rcm

mode = 0;
spot = 0;
time = 0;
figure();

fprintf(2,'\n ******* STARTING ******* \n');





%home_pose = [ 0.2245; 0.0315; -0.1934; 0 ; 0.2 ; 3.14/2 ];

%Calcolo home pose cercando dov'è l'ee_psm rispetto al nostro ecm_vision
home_pose = utilsECM.getPose(h_PSM,h_VS,ID,vrep);  %TARGET_POSE  TODO! ! !
disp(home_pose); %RITORNA TUTTI ZERI.... PERCHE NON VEDE IL PSM???
%MI SERVE CONOSCERE LE COORDS DEL PSM_EE ! ! !




while spot < 6 % spots are 5
    
    time = time +1;
    
    Q = kinematicsECM.getJoints(ID, vrep, h_joints);
    
    if mode == 0
        
        % 1) READ CURRENT POSE OF EE_ECM wtr VS_ECM
        
        [~, ee_position]=vrep.simxGetObjectPosition(ID, h_EE, h_VS, vrep.simx_opmode_streaming);
        [~, ee_orientation]=vrep.simxGetObjectOrientation(ID, h_EE, h_VS, vrep.simx_opmode_streaming);
        
        
        ee_pose= [ee_position, ee_orientation]';
        
        % 2) COMPUTE ERROR
        err = utilsECM.computeError(home_pose,ee_pose);
             
        % 3) EVALUATE EXIT CONDITION (just on position)
        if norm(err,2)< 10^-3           
            spot = spot+1;
            mode = 1;
            fprintf(1, 'GOING TOWARD SPOT : %d \n', spot);
            pause(1);
            
        end
        
        % 4) CORRECT AND UPDATE POSE
        
        % computing new configuration via inverse inverse kinematics
        Q = kinematicsECM.inverse_kinematics(Q,err,mode);
        
        % sending to joints
        [~] = vrep.simxSetJointPosition(ID, h_j1, Q(1), vrep.simx_opmode_streaming);
        [~] = vrep.simxSetJointPosition(ID, h_j2, Q(2), vrep.simx_opmode_streaming);
        [~] = vrep.simxSetJointPosition(ID, h_j3, Q(3), vrep.simx_opmode_streaming);
        [~] = vrep.simxSetJointPosition(ID, h_j4, Q(4), vrep.simx_opmode_streaming);
        
       
        
        pause(0.01);
        
        % PLOT
        if( mod(time,8)==0)
            
            x = time/100;
            y = norm(err(2:4),2);
            % plot(x,y,'--b');
            subplot(2,1,1)
            stem(x,y,'-k');
            ylim( [0 0.5]);
            xlabel('time')
            ylabel('norm error')
            title('Orientation error')
            hold on
            grid on
            
            subplot(2,1,2)
            y1 = norm(err(1:3),2);
            stem(x,y1,'-k');
            ylim( [0 0.25]);
            xlabel('time')
            ylabel('norm error')
            title('Position error')
            hold on
            grid on
            
        end
        
    elseif mode == 1
        break;
    end
end

fprintf(2,' \n **** PROCESS ENDED ***** \n');
disp("final pose :");
disp(ee_pose);

disp("direct kin position :");
disp(kinematicsECM.direct_kinematics(Q));

disp("absolute difference % :");
diff100 = ( kinematicsECM.direct_kinematics(Q) - ee_pose(1:3) )*100;
diff100 = round( diff100, 3);
disp(abs(diff100));
