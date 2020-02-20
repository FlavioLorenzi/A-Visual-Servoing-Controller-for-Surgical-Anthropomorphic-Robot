classdef utilsECM
    % utilities methods
    properties
        % null
    end
    methods (Static)
        
        function [clientID,vrep] = init_connection()
            
            % function used to initialize connection with vrep side
            
            fprintf(1,'START...  \n');
            vrep=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
            vrep.simxFinish(-1); % just in case, close all opened connections
            clientID=vrep.simxStart('127.0.0.1',19997,true,true,5000,5);
            fprintf(1,'client %d\n', clientID);
            if (clientID > -1)
                fprintf(1,'Connection: OK... \n');
                
            else
                fprintf(2,'Connection: ERROR \n');
                return;
            end
        end
        
        
        function [sync]  = syncronize(ID, vrep, h_joints, h_VS, h_EE, h_PSM)
            
            % used to wait to receive non zero values from vrep model
            
            % usually matlab and vrep need few seconds to send valid values
            vrep.simxAddStatusbarMessage(ID,' ', vrep.simx_opmode_oneshot);
            vrep.simxAddStatusbarMessage(ID,'------------------------------------------------', vrep.simx_opmode_oneshot);          
            vrep.simxAddStatusbarMessage(ID,'--- Matlab script CONNECTED ---', vrep.simx_opmode_oneshot);
            vrep.simxAddStatusbarMessage(ID,'------------------------------------------------', vrep.simx_opmode_oneshot);
            vrep.simxAddStatusbarMessage(ID,' ', vrep.simx_opmode_oneshot);

            sync = false;
            while ~sync
                % syncronizing all joints
                [~,~] = vrep.simxGetJointPosition(ID, h_joints(1), vrep.simx_opmode_streaming);
                [~,~] = vrep.simxGetJointPosition(ID, h_joints(2), vrep.simx_opmode_streaming);
                [~,~] = vrep.simxGetJointPosition(ID, h_joints(3), vrep.simx_opmode_streaming);
                [~,v] = vrep.simxGetJointPosition(ID, h_joints(4), vrep.simx_opmode_streaming);
                sync = norm(v,2)~=0;
            end
            
            sync = false;
            while ~sync
                % syncronizing position of EE_ECM wrt VS
                [~, v2]=vrep.simxGetObjectPosition(ID, h_EE, h_VS, vrep.simx_opmode_streaming);
                [~, ~]=vrep.simxGetObjectOrientation(ID, h_EE, h_VS, vrep.simx_opmode_streaming);
                
              % syncronizing position of PSM wrt ECM
                [~, ~]=vrep.simxGetObjectPosition(ID, h_PSM, h_VS, vrep.simx_opmode_streaming);
                [~, ~]=vrep.simxGetObjectOrientation(ID, h_PSM, h_VS, vrep.simx_opmode_streaming);
                
               
                
                sync = norm(v2,2)~=0;
                
            end
            
            sync = false;
            
            
           
            
            
        end
        
        
        function [J] = build_point_jacobian(u,v,z,fl)
            
            % function used to build INTERACTION matrix
            
            J = [ -fl/z     0          u/z     (u*v)/fl        -(fl+(u^2)/fl)      v; ...
                0         -fl/z      v/z     (fl+(v^2)/fl)    -(u*v)/fl          -u];
            
        
        
        end
        
        function [error] = computeError(desired, current)
            % computes error between poses
            % angdiff(a,b) computes b-a
            
            error = [desired(1:2)- current(1:2); angdiff(desired(3:4),current(3:4) )];
           
            
        end
        
        function [] = compute_grasp(clientID, h_7sx, h_7dx, vrep)
            % NOT USED
            
            % this function computes a grasp (only image rendering)
            % no interaction with objects
            
            sx = vrep.simxGetJointPosition(clientID,h_7sx,vrep.simx_opmode_streaming);
            dx = vrep.simxGetJointPosition(clientID,h_7dx,vrep.simx_opmode_streaming);
            
            % open
            while sx < 3.14/4
                [~] = vrep.simxSetJointPosition(clientID, h_7sx, sx, vrep.simx_opmode_streaming);
                sx = sx + 0.02;
                [~] = vrep.simxSetJointPosition(clientID, h_7dx, sx, vrep.simx_opmode_streaming);
                dx = dx + 0.02;
                pause(0.05);
            end
            
            pause(1);
            
            % close
            while sx > 0
                [~] = vrep.simxSetJointPosition(clientID, h_7sx, sx, vrep.simx_opmode_streaming);
                sx = sx - 0.02;
                [~] = vrep.simxSetJointPosition(clientID, h_7dx, sx, vrep.simx_opmode_streaming);
                dx = dx - 0.02;
                pause(0.05);
            end
        end
        
        function [pose] = getPose(who,wrt_who,ID,vrep)
            
            % reading where's who 
            [~, position] = vrep.simxGetObjectPosition(ID, who, wrt_who, vrep.simx_opmode_streaming);
            [~, orientation] = vrep.simxGetObjectOrientation(ID, who, wrt_who, vrep.simx_opmode_streaming);
            pose = [position, orientation]';
            
        end
        
        function [] = setPose(pose,who,wrt_who,ID,vrep)
            
            % setting pose of handle who
            [~]= vrep.simxSetObjectPosition(ID, who, wrt_who, pose(1:3), vrep.simx_opmode_oneshot);
            [~]= vrep.simxSetObjectOrientation(ID, who, wrt_who, pose(4:6), vrep.simx_opmode_oneshot);
            
        end
        
        function [us_desired, vs_desired] = getLandmarksPosition(ID, vrep, h_VS, h_L,fl)
            
            % preallocating for speed
            us_desired = zeros(4,5);
            vs_desired = zeros(4,5);
                       
            % desired features EXTRACTION
            sync = false; % to be fixed
            
            for b=1:4 % balls
                for s=1:5 % spots
                    while ~sync % until i dont get valid values
                        [~, l_position]=vrep.simxGetObjectPosition(ID, h_L(b,s), h_VS, vrep.simx_opmode_streaming);
                        sync = norm(l_position,2)~=0;
                    end
                    sync=false;
                    
                    % here you have all landmark positions in image plane (perspective)
                    us_desired(b,s)= fl*l_position(1)/l_position(3);
                    vs_desired(b,s)= fl*l_position(2)/l_position(3);
                    
                end
            end
        end
        
        function [us_ee, vs_ee, zs_ee] = get_EE_LandmarksPosition(ID, vrep, h_VS, h_L_EE, fl)
            
            %% TO BE FIXED (NOT YET INSERTED)
            
            us_ee = zeros(4,1);
            vs_ee = zeros(4,1);
            zs_ee = zeros(4,1);
            
            
            sync = false; % to be fixed
            
            % GETTING CURRECT POSITION OF EE IN IMAGE PLANE
            for b=1:4 % ee_balls
                while ~sync  % until i dont get valid values
                    [~, l_position]=vrep.simxGetObjectPosition(ID, h_L_EE(b), h_VS, vrep.simx_opmode_streaming);
                    sync = norm(l_position,2)~=0;
                end
                
                zs_ee(b)= l_position(3);
                us_ee(b)= fl*l_position(1)/l_position(3);
                vs_ee(b)= fl*l_position(2)/l_position(3);
                
            end
            
        end
        
        
    end
end