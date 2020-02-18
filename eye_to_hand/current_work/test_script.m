%--------------------------------------------------------------------------
%   INIT STUFF

cd(fileparts(mfilename('fullpath')));
clear;
close all;
clc;

pause(2);
%--------------------------------------------------------------------------
% CONNECTION TO VREP

[ID,vrep] = utils.init_connection();

% COLLECTING HANDLES

%--------------------------------------------------------------------------

% vision sensor
[~, h_VS] =vrep.simxGetObjectHandle(ID, 'ECM_view', vrep.simx_opmode_blocking);
%--------------------------------------------------------------------------

% end effector
[~, h_EE]=vrep.simxGetObjectHandle(ID, 'EE', vrep.simx_opmode_blocking);

% dummy followed
[~, h_Dummy]=vrep.simxGetObjectHandle(ID, 'FollowedDummy', vrep.simx_opmode_blocking);
%--------------------------------------------------------------------------

% force sensor
[~, h_FS]=vrep.simxGetObjectHandle(ID, 'Force_sensor', vrep.simx_opmode_blocking);
%--------------------------------------------------------------------------

% rcm handle
[~, h_RCM]=vrep.simxGetObjectHandle(ID, 'RCM_PSM1', vrep.simx_opmode_blocking);

% reference for direct kin
% first RRP joints
[~, h_j1] = vrep.simxGetObjectHandle(ID,'J1_PSM1',vrep.simx_opmode_blocking);
[~, h_j2] = vrep.simxGetObjectHandle(ID,'J2_PSM1',vrep.simx_opmode_blocking);
[~, h_j3] = vrep.simxGetObjectHandle(ID,'J3_PSM1',vrep.simx_opmode_blocking);

% second RRR joints
[~, h_j4] = vrep.simxGetObjectHandle(ID,'J1_TOOL1',vrep.simx_opmode_blocking);
[~, h_j5] = vrep.simxGetObjectHandle(ID,'J2_TOOL1',vrep.simx_opmode_blocking);
[~, h_j6] = vrep.simxGetObjectHandle(ID,'J3_TOOL1',vrep.simx_opmode_blocking);

%--------------------------------------------------------------------------

% collection of all joint handles
h_Joints = [h_j1; h_j2; h_j3; h_j4; h_j5; h_j6];

sync = utils.syncronize(ID, vrep, h_Joints, h_RCM, h_VS, h_Dummy, h_EE, h_FS);

if sync
    fprintf(1,'Sycronization: OK... \n');
    pause(1);
end

%--------------------------------------------------------------------------

% landmarks attached to goal positions :
% we have 5 location to achieve
% each location has 4 landmarks

h_L = zeros(4,5); % here i save handles of landmarks

% landmarks attached to spots
for b=1:4 % each spot has 4 balls
    for s=1:5 % 5 total spots
        [~, h_L(b,s)]=vrep.simxGetObjectHandle(ID, ['Landmark', num2str(s), num2str(b)], vrep.simx_opmode_blocking);
    end
end

h_L_EE = zeros(4,5); % here i save handles of balls attacched to EE

% landmarks attached to DUMMY FOLLOWED -> 'LandmarkEE1,2,3,4'
for b=1:4
    [~, h_L_EE(b)]=vrep.simxGetObjectHandle(ID, ['LandmarkEE', num2str(b)], vrep.simx_opmode_blocking);
end

%__________________________________________________________________________

%   SETTINGS
%__________________________________________________________________________

% focal length (depth of the near clipping plane)
fl = 0.01;

% control gain in mode 1 (see below)
% K = eye(6)*(10^-2)*1.5;
% after tuning
K = diag( 0.8*[0.2 0.2 0.2 3.5 3.5 5.5]*10^-1);

% control gain in mode 0 (see below)
H = diag( [1 1 1 0.1 0.1 0.1]*10^-1);

% compliance matrix of manipulator
C = diag( [0.1 0.1 0.1 0 0 0]*1);

% preallocating for speed
% us_desired = zeros(4,5);
% vs_desired = zeros(4,5);
us_ee = zeros(4,1);
vs_ee = zeros(4,1);
zs_ee = zeros(4,1);

% null desired force and torque
force_torque_d=zeros(6,1);

% % desired features EXTRACTION
% sync=false;
% for b=1:4 % balls
%     for s=1:5 % spots
%         while ~sync % until i dont get valid values
%             [~, l_position]=vrep.simxGetObjectPosition(ID, h_L(b,s), h_VS, vrep.simx_opmode_streaming);
%             sync = norm(l_position,2)~=0;
%         end
%         sync=false;
%
%         % here you have all landmark positions in image plane (perspective)
%         us_desired(b,s)= fl*l_position(1)/l_position(3);
%         vs_desired(b,s)= fl*l_position(2)/l_position(3);
%
%     end
% end

sync=false;
[us_desired, vs_desired] = utils.getLandmarksPosition(ID, vrep, h_VS, h_L, fl);

home_pose_wrt_world = [ -1.54;   -4.069e-2;    +7.25e-1;  -1.79e+2; -4.2305e-02;         6.80022e-01];
% sending new pose of dummy attached to EE wrt world
utils.setPose(home_pose_wrt_world,h_Dummy,-1,ID,vrep);

% starting from zero config
kinematicsRCM.setJoints(ID, vrep, h_Joints, zeros(6,1));
pause(3);

% getting home_pose wrt RCM for mode 0
home_pose_wrt_RCM = utils.getPose(h_Dummy,h_RCM,ID,vrep);

%__________________________________________________________________________
%__________________________________________________________________________

%	PROCESS LOOP
%__________________________________________________________________________
%__________________________________________________________________________

% position and orientation of RCM wrt VS (should be costant)
% used in conversion of coordinates from VS to RCM

vs2rcm = utils.getPose(h_RCM,h_VS,ID,vrep);

%start from landmark at spot+1
spot = 0; % round(rand*5)
% first go to home pose in mode 0
mode = 0;
% time variable useful for plot ecc.
time = 0;
% if ghost reached position of landmark (ghost is the dummy)
ghost_reached = false;

fprintf(2,'******* STARTING ******* \n');

% used for the final plot

% x_coord = zeros(1, 16000);
% y_coord = zeros(1, 16000);
% z_coord = zeros(1, 16000);
pause(0.1);


while spot<6
        
    while mode==1
        
        time = time +1;
               
        %__________________________________________________________________
        
        %	1) FEATURES and DEPTH EXTRACTION
        %__________________________________________________________________
        
        % GETTING CURRECT POSITION OF EE IN IMAGE PLANE
        for b=1:4 % ee_balls
            while ~sync  % until i dont get valid values
                [~, l_position]=vrep.simxGetObjectPosition(ID, h_L_EE(b), h_VS, vrep.simx_opmode_streaming);
                sync = norm(l_position,2)~=0;
            end
            sync=false;
            
            zs_ee(b)= l_position(3);
            us_ee(b)= fl*l_position(1)/l_position(3);
            vs_ee(b)= fl*l_position(2)/l_position(3);
            
        end
        
        % TO-DO
        % [us_ee, vs_ee, zs_ee] = utils.get_EE_LandmarksPosition(ID, vrep, h_VS, h_L_EE, fl);
        
        
        
        %__________________________________________________________________
        
        %	2) BUILDING the IMAGE JACOBIAN and COMPUTING THE ERROR (vision-based only)
        %__________________________________________________________________
        
        % building the jacobian
        L = [ utils.build_point_jacobian(us_ee(1),vs_ee(1),zs_ee(1),fl); ...
            utils.build_point_jacobian(us_ee(2),vs_ee(2),zs_ee(2),fl); ...
            utils.build_point_jacobian(us_ee(3),vs_ee(3),zs_ee(3),fl); ...
            utils.build_point_jacobian(us_ee(4),vs_ee(4),zs_ee(4),fl)];
        
        % computing the error
        image_error= [us_desired(1,spot)-us_ee(1); ...
            vs_desired(1,spot)-vs_ee(1); ...
            us_desired(2,spot)-us_ee(2); ...
            vs_desired(2,spot)-vs_ee(2); ...
            us_desired(3,spot)-us_ee(3); ...
            vs_desired(3,spot)-vs_ee(3); ...
            us_desired(4,spot)-us_ee(4); ...
            vs_desired(4,spot)-vs_ee(4)];
        
        if norm(image_error,2)<10^-5 && ghost_reached ==false
            ghost_reached = true;
            %disp("Ghost has reached desired spot");
            %COLOR GREEN for the visited landmark
            [return_code, spot_output, ~, ~, ~] = vrep.simxCallScriptFunction(ID, ['L_Prismatic_joint'] ,vrep.sim_scripttype_childscript(),'changeColorGreen',[spot],[],[],[],vrep.simx_opmode_blocking);
         
            
        end
        
        %__________________________________________________________________
        
        %	3) ADJUSTING the ERROR (via the force-based infos)
        %__________________________________________________________________
        
        [~, ~, force, torque] = vrep.simxReadForceSensor(ID, h_FS, vrep.simx_opmode_streaming);
        
        force_torque=[force'; torque'];
        % force_torque=round(force_torque,2);
        
        force_correction = L*C*(force_torque_d-force_torque);
        
        error = image_error + force_correction;
        
        time = time +1;
        
        %__________________________________________________________________
        
        %	4) COMPUTING the EE DISPLACEMENT
        %__________________________________________________________________
        
        % computing the displacement
        ee_displacement = K*pinv(-L)*error;
        
        if norm(ee_displacement,2)<10^-2.5 %10^-2.9
            ee_displacement = (ee_displacement/norm(ee_displacement,2))*10^-2.5;
        end
        
        %__________________________________________________________________
        
        %	5) UPDATING THE POSE
        %__________________________________________________________________
        
        % getting the current pose wrt VS
        ee_pose_VS = utils.getPose(h_Dummy,h_VS,ID,vrep);
        
        % getting the new pose
        next_ee_pose_VS = ee_pose_VS + ee_displacement;
        
        % getting next pose wrt RCM
        next_ee_wrt_RCM = utils.getPoseInRCM(vs2rcm, next_ee_pose_VS);
        
        utils.setPose(next_ee_wrt_RCM,h_Dummy,h_RCM,ID,vrep);
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %                                                               %%
        %        HERE YOU FOLLOW DUMMY USING INVERSE KINEMATICS         %%
        %                                                               %%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        % gettting pose of dummy and EE
        dummy_pose = utils.getPose(h_Dummy,h_RCM,ID,vrep);
        ee_wrt_RCM = utils.getPose(h_EE, h_RCM,ID,vrep);
        
        % computing error
        distance2dummy = utils.computeError(dummy_pose,ee_wrt_RCM);
        
        Q = kinematicsRCM.getJoints(ID, vrep, h_Joints);
        
        % computing new configuration via inverse inverse kinematics
        Q = kinematicsRCM.inverse_kinematics(Q,distance2dummy,0);
        
        % sending to joints
        kinematicsRCM.setJoints(ID, vrep, h_Joints, Q);
        
        % data saved for plot
        x_coord(time) = ee_wrt_RCM(1);
        y_coord(time) = ee_wrt_RCM(2);
        z_coord(time) = ee_wrt_RCM(3);
        force(time) = norm(force_correction,2);
        total_error(time) = norm(error,2);
        
        % evaluating exit condition
        if norm(distance2dummy(1:3),2) <= 10^-3 && ghost_reached
            mode =0;
            fprintf(1,'********** OK ********** \n');
            pause(3);
            ghost_reached = false;
            time = 0;
            
            % plot EE position during last process
            % PlotData.plot_EE(spot,[x_coord],[y_coord],[z_coord]);
            
            % plot of force and image error during time for last process
            % PlotData.plot_ForceAndImageErr(spot, [force], [total_error]);
                        
        end   
        
        if(mod(time,10)==0)
            PlotData.plot_image_error(spot, us_ee, vs_ee, us_desired, vs_desired);
        end
        
    end
    
    while mode==0
        %%
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %                                                               %%
        %        HERE YOU FOLLOW DUMMY USING INVERSE KINEMATICS         %%
        %                                                               %%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        dummy_pose = utils.getPose(h_Dummy,h_RCM,ID,vrep);
        ee_wrt_RCM = utils.getPose(h_EE, h_RCM,ID,vrep);
        
        error = utils.computeError(home_pose_wrt_RCM,dummy_pose);
        
        % computing the displacement
        displacement = H*error;
        
        % updating the pose
        dummy_pose = dummy_pose + displacement;
        
        utils.setPose(dummy_pose, h_Dummy, h_RCM, ID, vrep);
        
        % COMPUTE ERROR
        distance2dummy = utils.computeError(utils.getPose(h_Dummy,h_RCM,ID,vrep),utils.getPose(h_EE, h_RCM,ID,vrep));
        
        Q = kinematicsRCM.getJoints(ID, vrep, h_Joints);
        
        % computing new configuration via inverse inverse kinematics
        Q = kinematicsRCM.inverse_kinematics(Q,distance2dummy,0);
        
        kinematicsRCM.setJoints(ID, vrep, h_Joints, Q);
        
        if(max(error)<=0.001 && norm(distance2dummy(1:3),2) <= 0.01)
            spot = spot+1;
            
            %Restore initial colors of landmarks
            [return_code, spot_output, ~, ~, ~] = vrep.simxCallScriptFunction(ID, ['L_Prismatic_joint'] ,vrep.sim_scripttype_childscript(),'restoreColors',[spot],[],[],[],vrep.simx_opmode_blocking);
            
            if spot>5
                
                break
            end
            
            %RED COLOR for the next landmark
            [return_code, spot_output, ~, ~, ~] = vrep.simxCallScriptFunction(ID, ['L_Prismatic_joint'] ,vrep.sim_scripttype_childscript(),'changeColorRed',[spot],[],[],[],vrep.simx_opmode_blocking);
            fprintf(1,'GOING TOWARD LANDMARK: %d \n',spot);
            pause(2);
            
            mode=1;
            
            figure(spot);
            
            
        end
        
    end
    
end

fprintf(2,'**** PROCESS ENDED ***** \n');