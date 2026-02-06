% Lauren Ervin
% 1/8/26
% Tensegrity snake Vicon motion tracking and plotting script
%
% Input:
% Read in tensegrity snake XYZ coordinates from corresponding CSV file.
% Make sure CSV filename is correct and set the final row number as well as
% the last row number for each of the 6 nodes if different.
% Also uncomment video setup on lines 28-32, comment line 33, and set 
% appropriate name for video output if wanted.
% Output:
% Generate motor endpoints Ei, midpoints Mi, and inner points Ii for six
% modules.  From this, plot movement of tensegrity snake through sequence.
% Also return distance information for each outer node.



% Motor 1 is rows 6 - 915
% Motor 2 is rows 917 - 1779
% Motor 3 is rows 1781 - 2786
% Motor 1 + 2 is rows 2788 - 3912
% Motor 1 + 3 is rows 3914 - 4865
% Motor 2 + 3 is rows 4867 - 6027
M = readmatrix('curvature_combined.csv','Range','C6:AO6027');
final_row = 6017;

% Set start/stop time for different curvature runs
Motor1Start = 1;
Motor1End = 901; %905
Motor2Start = 921;
Motor2End = 1761;
Motor3Start = 1781;
Motor3End = 2771;
Motor1and2Start = 2781;
Motor1and2End = 3901;
Motor1and3Start = 3911;
Motor1and3End = 4851;
Motor2and3Start = 4861;
Motor2and3End = 6011;

% Set last row that each node contains Vicon data
lastE1 = final_row;
lastE2 = final_row;
lastE3 = final_row;
lastE4 = final_row;
lastE5 = final_row;
lastE6 = final_row;

% Generate video info
video_title = "curvature_tracking.avi"; %Update with descriptive title and make sure not to overwrite previous video
v = VideoWriter(video_title, "Uncompressed AVI");
v.FrameRate = 15; %Set video frame rate
open(v);
run_video = 1;
% run_video = 0;

% Gather all motor endpoints
E1 = M(1:final_row, 1:3);
E2 = M(1:final_row, 13:15);
E3 = M(1:final_row, 25:27);

% Gather all ???
V1 = M(1:final_row, 4:6);
V2 = M(1:final_row, 16:18);
V3 = M(1:final_row, 28:30);

% Gather all midpoints
M1 = M(1:final_row, 7:9);
M2 = M(1:final_row, 19:21);
M3 = M(1:final_row, 31:33);


% Gather all inner points
I1 = M(1:final_row, 10:12);
I2 = M(1:final_row, 22:24);
I3 = M(1:final_row, 34:36);

% Top stationary plane?
Plane = M(1:final_row, 37:39);


% Initialize tracking variables for plotting purposes
EndpointsMod1Tracking = zeros(1, 3, final_row);
EndpointsMod2Tracking = zeros(1, 3, final_row);
EndpointsMod1TrackingColor = zeros(1, final_row);
EndpointsMod2TrackingColor = zeros(1, final_row);


% Cute colors
% ("#C7A491")
% ("#EECFCA")
% ("#997B66")
% ("#919682")
% ("#C7CDBF")
% ("#595E48")

% Plot overall distance + path traveled for each of the 6 curvatures
plotCurvature(v, run_video, Motor1Start, Motor1End, "#C7A491", E1, E2, E3, M1, M2, M3, I1, I2, I3)
plotCurvature(v, run_video, Motor2Start, Motor2End, "#EECFCA", E1, E2, E3, M1, M2, M3, I1, I2, I3)
plotCurvature(v, run_video, Motor3Start, Motor3End, "#997B66", E1, E2, E3, M1, M2, M3, I1, I2, I3)
plotCurvature(v, run_video, Motor1and2Start, Motor1and2End, "#919682", E1, E2, E3, M1, M2, M3, I1, I2, I3)
plotCurvature(v, run_video, Motor1and3Start, Motor1and3End, "#C7CDBF", E1, E2, E3, M1, M2, M3, I1, I2, I3)
plotCurvature(v, run_video, Motor2and3Start, Motor2and3End, "#595E48", E1, E2, E3, M1, M2, M3, I1, I2, I3)

if run_video == 1
    close(v)
end

% Plot end position
figure(2)
tiledlayout(3,2);
plotCurvatureFinal(351, "#C7A491", E1, E2, E3, M1, M2, M3, I1, I2, I3)
plotCurvatureFinal(1261, "#EECFCA", E1, E2, E3, M1, M2, M3, I1, I2, I3)
plotCurvatureFinal(2401, "#997B66", E1, E2, E3, M1, M2, M3, I1, I2, I3)
plotCurvatureFinal(3191, "#919682", E1, E2, E3, M1, M2, M3, I1, I2, I3)
plotCurvatureFinal(4331, "#C7CDBF", E1, E2, E3, M1, M2, M3, I1, I2, I3)
plotCurvatureFinal(5271, "#595E48", E1, E2, E3, M1, M2, M3, I1, I2, I3)


% Plot different gaits within movement
% Manually set time that different gaits occur in a movement sequence
figure(3)
t = tiledlayout(2,2);

for i = [1, 250, 700, 1080]
    nexttile;
    % For module 1 since we have all points, we have to end at the starting node (1)
    EndpointsMod1 = [E1(i,:); E2(i,:); E3(i,:); E1(i,:)];
    fill3(EndpointsMod1(:,1),EndpointsMod1(:,2),EndpointsMod1(:,3), 'g-o')
    hold on
    MidpointsMod1 = [M1(i,:); M2(i,:); M3(i,:); M1(i,:)];
    fill3(MidpointsMod1(:,1),MidpointsMod1(:,2),MidpointsMod1(:,3), 'c-o')
    hold on
    InnerpointsMod1 = [I1(i,:); I2(i,:); I3(i,:); I1(i,:)];
    fill3(InnerpointsMod1(:,1),InnerpointsMod1(:,2),InnerpointsMod1(:,3), 'm-o')
    hold on

    % For module 2 since we have all points, we have to end at the starting node (4)
    EndpointsMod2 = [E4(i,:); E5(i,:); E6(i,:); E4(i,:)];
    fill3(EndpointsMod2(:,1),EndpointsMod2(:,2),EndpointsMod2(:,3), 'b-o')
    hold on
    MidpointsMod2 = [M4(i,:); M5(i,:); M6(i,:); M4(i,:)];
    fill3(MidpointsMod2(:,1),MidpointsMod2(:,2),MidpointsMod2(:,3), 'r-o')
    hold on
    InnerpointsMod2 = [I4(i,:); I5(i,:); I6(i,:); I4(i,:)];
    fill3(InnerpointsMod2(:,1),InnerpointsMod2(:,2),InnerpointsMod2(:,3), 'y-o')
    hold on

    % Tendons 1 - 6 since we're missing node 1 points
    Tendon1 = [E1(i,:); M1(i,:); I1(i,:)];
    plot3(Tendon1(:,1),Tendon1(:,2),Tendon1(:,3), 'w-o', 'LineWidth', 2, 'MarkerEdgeColor','k')
    hold on
    Tendon2 = [E2(i,:); M2(i,:); I2(i,:)];
    plot3(Tendon2(:,1),Tendon2(:,2),Tendon2(:,3), 'w-o', 'LineWidth', 2, 'MarkerEdgeColor','k')
    hold on
    Tendon3 = [E3(i,:); M3(i,:); I3(i,:)];
    plot3(Tendon3(:,1),Tendon3(:,2),Tendon3(:,3), 'w-o', 'LineWidth', 2, 'MarkerEdgeColor','k')
    hold on
    Tendon4 = [E4(i,:); M4(i,:); I4(i,:)];
    plot3(Tendon4(:,1),Tendon4(:,2),Tendon4(:,3), 'w-o', 'LineWidth', 2, 'MarkerEdgeColor','k')
    hold on
    Tendon5 = [E5(i,:); M5(i,:); I5(i,:)];
    plot3(Tendon5(:,1),Tendon5(:,2),Tendon5(:,3), 'w-o', 'LineWidth', 2, 'MarkerEdgeColor','k')
    hold on
    Tendon6 = [E6(i,:); M6(i,:); I6(i,:)];
    plot3(Tendon6(:,1),Tendon6(:,2),Tendon6(:,3), 'w-o', 'LineWidth', 2, 'MarkerEdgeColor','k')
    hold on
    if i == 1
        title('Base State');
    elseif i == 250 || i == 700
        title('State 1');
    else
        title('State 2');
    end
end
title(t, 'Undulation Gait');


% Track two endpoints throughout gait movement
% Manually set time that different gaits occur in a movement sequence
figure(3)
for i = [1, final_row] % Plot starting and ending full robot configurations
    % For module 1 since we have all points, we have to end at the starting node (1)
    EndpointsMod1 = [E1(i,:); E2(i,:); E3(i,:); E1(i,:)];
    fill3(EndpointsMod1(:,1),EndpointsMod1(:,2),EndpointsMod1(:,3), 'g-o')
    hold on
    MidpointsMod1 = [M1(i,:); M2(i,:); M3(i,:); M1(i,:)];
    fill3(MidpointsMod1(:,1),MidpointsMod1(:,2),MidpointsMod1(:,3), 'c-o')
    hold on
    InnerpointsMod1 = [I1(i,:); I2(i,:); I3(i,:); I1(i,:)];
    fill3(InnerpointsMod1(:,1),InnerpointsMod1(:,2),InnerpointsMod1(:,3), 'm-o')
    hold on

    % For module 2 since we have all points, we have to end at the starting node (4)
    EndpointsMod2 = [E4(i,:); E5(i,:); E6(i,:); E4(i,:)];
    fill3(EndpointsMod2(:,1),EndpointsMod2(:,2),EndpointsMod2(:,3), 'b-o')
    hold on
    MidpointsMod2 = [M4(i,:); M5(i,:); M6(i,:); M4(i,:)];
    fill3(MidpointsMod2(:,1),MidpointsMod2(:,2),MidpointsMod2(:,3), 'r-o')
    hold on
    InnerpointsMod2 = [I4(i,:); I5(i,:); I6(i,:); I4(i,:)];
    fill3(InnerpointsMod2(:,1),InnerpointsMod2(:,2),InnerpointsMod2(:,3), 'y-o')
    hold on

    % Tendons 1 - 6 since we're missing node 1 points
    Tendon1 = [E1(i,:); M1(i,:); I1(i,:)];
    plot3(Tendon1(:,1),Tendon1(:,2),Tendon1(:,3), 'w-o', 'LineWidth', 2, 'MarkerEdgeColor','k')
    hold on
    Tendon2 = [E2(i,:); M2(i,:); I2(i,:)];
    plot3(Tendon2(:,1),Tendon2(:,2),Tendon2(:,3), 'w-o', 'LineWidth', 2, 'MarkerEdgeColor','k')
    hold on
    Tendon3 = [E3(i,:); M3(i,:); I3(i,:)];
    plot3(Tendon3(:,1),Tendon3(:,2),Tendon3(:,3), 'w-o', 'LineWidth', 2, 'MarkerEdgeColor','k')
    hold on
    Tendon4 = [E4(i,:); M4(i,:); I4(i,:)];
    plot3(Tendon4(:,1),Tendon4(:,2),Tendon4(:,3), 'w-o', 'LineWidth', 2, 'MarkerEdgeColor','k')
    hold on
    Tendon5 = [E5(i,:); M5(i,:); I5(i,:)];
    plot3(Tendon5(:,1),Tendon5(:,2),Tendon5(:,3), 'w-o', 'LineWidth', 2, 'MarkerEdgeColor','k')
    hold on
    Tendon6 = [E6(i,:); M6(i,:); I6(i,:)];
    plot3(Tendon6(:,1),Tendon6(:,2),Tendon6(:,3), 'w-o', 'LineWidth', 2, 'MarkerEdgeColor','k')
    hold on
end

% for i = 1:3:final_row
%     EndpointsMod1Tracking = (E1(i,:) + E2(i,:) + E3(i,:))/3;
%     EndpointsMod2Tracking = (E4(i,:) + E5(i,:) + E6(i,:))/3;
% 
%     EndpointsMod1TrackingColor = 0 + (i/final_row);
%     EndpointsMod2TrackingColor = 0 + (i/final_row);
% 
%     plot3(EndpointsMod1Tracking(:,1), EndpointsMod1Tracking(:,2), EndpointsMod1Tracking(:,3), 'x', 'Color', [0 0 EndpointsMod1TrackingColor], 'MarkerSize', 15, 'LineWidth', 3)
%     hold on
%     plot3(EndpointsMod2Tracking(:,1), EndpointsMod2Tracking(:,2), EndpointsMod2Tracking(:,3), 'x', 'Color', [EndpointsMod2TrackingColor 0 0], 'MarkerSize', 15, 'LineWidth', 3)
%     hold on
% end
%view(0,0)
title(t, 'Full Inchworm Gait Tracked');


% Figuring out reference frame stuff
figure(4)
for i = [1, 200, 201, 480, 481, 670, 671, 900, 901, 1120, 1121, 1320, 1321, 1460, 1461, 1720, 1721, 1950, 1951, 2150, 2151, 2340, 2341, 2580, 2581, 2730, 2731, 2930, 2931, 3080, 3081, 3320, 3321, 3520, 3521, 3720, 3721, 3860, 3861, 4070, 4071, 4310, 4311, 4520, 4521, 4700, 4701, 4890] % Plot starting and ending full robot configurations
    % For module 1 since we have all points, we have to end at the starting node (1)
    EndpointsMod1 = [E1(i,:); E2(i,:); E3(i,:); E1(i,:)];
    fill3(EndpointsMod1(:,1),EndpointsMod1(:,2),EndpointsMod1(:,3), 'g-o')
    hold on
    MidpointsMod1 = [M1(i,:); M2(i,:); M3(i,:); M1(i,:)];
    fill3(MidpointsMod1(:,1),MidpointsMod1(:,2),MidpointsMod1(:,3), 'c-o')
    hold on
    InnerpointsMod1 = [I1(i,:); I2(i,:); I3(i,:); I1(i,:)];
    fill3(InnerpointsMod1(:,1),InnerpointsMod1(:,2),InnerpointsMod1(:,3), 'm-o')
    hold on

    % For module 2 since we have all points, we have to end at the starting node (4)
    EndpointsMod2 = [E4(i,:); E5(i,:); E6(i,:); E4(i,:)];
    fill3(EndpointsMod2(:,1),EndpointsMod2(:,2),EndpointsMod2(:,3), 'b-o')
    hold on
    MidpointsMod2 = [M4(i,:); M5(i,:); M6(i,:); M4(i,:)];
    fill3(MidpointsMod2(:,1),MidpointsMod2(:,2),MidpointsMod2(:,3), 'r-o')
    hold on
    InnerpointsMod2 = [I4(i,:); I5(i,:); I6(i,:); I4(i,:)];
    fill3(InnerpointsMod2(:,1),InnerpointsMod2(:,2),InnerpointsMod2(:,3), 'y-o')
    hold on

    % Tendons 1 - 6 since we're missing node 1 points
    Tendon1 = [E1(i,:); M1(i,:); I1(i,:)];
    plot3(Tendon1(:,1),Tendon1(:,2),Tendon1(:,3), 'w-o', 'LineWidth', 2, 'MarkerEdgeColor','k')
    hold on
    Tendon2 = [E2(i,:); M2(i,:); I2(i,:)];
    plot3(Tendon2(:,1),Tendon2(:,2),Tendon2(:,3), 'w-o', 'LineWidth', 2, 'MarkerEdgeColor','k')
    hold on
    Tendon3 = [E3(i,:); M3(i,:); I3(i,:)];
    plot3(Tendon3(:,1),Tendon3(:,2),Tendon3(:,3), 'w-o', 'LineWidth', 2, 'MarkerEdgeColor','k')
    hold on
    Tendon4 = [E4(i,:); M4(i,:); I4(i,:)];
    plot3(Tendon4(:,1),Tendon4(:,2),Tendon4(:,3), 'w-o', 'LineWidth', 2, 'MarkerEdgeColor','k')
    hold on
    Tendon5 = [E5(i,:); M5(i,:); I5(i,:)];
    plot3(Tendon5(:,1),Tendon5(:,2),Tendon5(:,3), 'w-o', 'LineWidth', 2, 'MarkerEdgeColor','k')
    hold on
    Tendon6 = [E6(i,:); M6(i,:); I6(i,:)];
    plot3(Tendon6(:,1),Tendon6(:,2),Tendon6(:,3), 'w-o', 'LineWidth', 2, 'MarkerEdgeColor','k')
    hold on

    % Calculate frame based off orthonormal basis vectors
    [uhat, vhat, what, origin] = movement(E1(i,:), E2(i,:), E3(i,:), final_row);

    %Calculate module 2 frame based off orthonormal basis vectors
    [uhat2, vhat2, what2, origin2] = movement(E4(i,:), E5(i,:), E6(i,:), final_row);

    % Calculate rotation and translation from these orthonormal basis vectors
    if i == 1 || i == 201 || i == 481 || i == 671 || i == 901 || i == 1121 || i == 1321 || i == 1461 || i == 1721 || i == 1951 || i == 2151 || i == 2341 || i == 2581 || i == 2731 || i == 2931 || i == 3081 || i == 3321 || i == 3521 || i == 3721 || i == 3861 || i == 4071 || i == 4311 || i == 4521 || i == 4701
        rotation1 = [uhat'; what'; vhat'];
        translation1 = origin;
        transformation1 = [rotation1, translation1; 0,0,0,1];

        rotation1end2 = [uhat2'; what2'; vhat2'];
        translation1end2 = origin2;
        transformation1end2 = [rotation1end2, translation1end2; 0,0,0,1];
    end

    if i == 200 || i == 480 || i == 670 || i == 900 || i == 1120 || i == 1320 || i == 1460 || i == 1720 || i == 1950 || i == 2150 || i == 2340 || i == 2580 || i == 2730 || i == 2930 || i == 3080 || i == 3320 || i == 3520 || i == 3720 || i == 3860 || i == 4070 || i == 4310 || i == 4520 || i == 4700 || i == 4890
        rotation2 = [uhat'; what'; vhat'];
        translation2 = origin;
        transformation2 = [rotation2, translation2; 0,0,0,1];

        rotation2end2 = [uhat2'; what2'; vhat2'];
        translation2end2 = origin2;
        transformation2end2 = [rotation2end2, translation2end2; 0,0,0,1];

        T_rel = inv(transformation2)*transformation1;
        t_rel = T_rel(1:3, 4);
        d1 = norm(t_rel);
        R_rel = T_rel(1:3, 1:3);

        T_rel2 = inv(transformation2end2)*transformation1end2;
        t_rel2 = T_rel2(1:3, 4);
        d2 = norm(t_rel2);
        R_rel2 = T_rel2(1:3, 1:3);

        % 4. Convert Rotation to human-readable format (e.g., Euler Angles or Axis-Angle)
        euler_rel = rad2deg(rotm2eul(R_rel, 'ZYX')); % Z-Y-X convention
        axisang = rotm2axang(R_rel); % Axis of rotation and total angle
        angle_deg = rad2deg(axisang(4));

        euler_rel2 = rad2deg(rotm2eul(R_rel2, 'ZYX')); % Z-Y-X convention
        axisang2 = rotm2axang(R_rel2); % Axis of rotation and total angle
        angle_deg2 = rad2deg(axisang2(4));
        
        % Display Results
        fprintf('Relative Translation: %.2f and %.2f\n', d1, d2);
        %fprintf('Relative Euler Angles (deg): [%.2f, %.2f, %.2f] and [%.2f, %.2f, %.2f]\n', euler_rel, euler_rel2);
        fprintf('Total Rotation Angle: %.2f degrees and %.2f degrees\n', angle_deg, angle_deg2);
    end

    q_scale = 50; % Adjust length for visibility
    quiver3(origin(1), origin(2), origin(3), uhat(1), uhat(2), uhat(3), q_scale, 'r', 'LineWidth', 5, 'MaxHeadSize', 2);
    quiver3(origin(1), origin(2), origin(3), vhat(1), vhat(2), vhat(3), q_scale, 'g', 'LineWidth', 5, 'MaxHeadSize', 2);
    quiver3(origin(1), origin(2), origin(3), what(1), what(2), what(3), q_scale, 'b', 'LineWidth', 5, 'MaxHeadSize', 2);

    quiver3(origin2(1), origin2(2), origin2(3), uhat2(1), uhat2(2), uhat2(3), q_scale, 'r', 'LineWidth', 5, 'MaxHeadSize', 2);
    quiver3(origin2(1), origin2(2), origin2(3), vhat2(1), vhat2(2), vhat2(3), q_scale, 'g', 'LineWidth', 5, 'MaxHeadSize', 2);
    quiver3(origin2(1), origin2(2), origin2(3), what2(1), what2(2), what2(3), q_scale, 'b', 'LineWidth', 5, 'MaxHeadSize', 2);
    hold on

end
title(t, 'C 3D Movement');


% [rotx1, roty1, rotz1, t1] = tracking(E1, E2, E3, final_row);
% [rotx2, roty2, rotz2, t2] = tracking(E4, E5, E6, final_row);
% 
% 
% fprintf('Mid of E1, E2, E3 rotated %.2d%c, %.2d%c, %.2d%c in X, Y, and Z, and translated %.2dmm in X, %.2dmm in Y, %.2dmm in Z\n', round(rotx1), char(176), round(roty1), char(176), round(rotz1), char(176), round(t1(1)), round(t1(2)), round(t1(3)))
% fprintf('Mid of E4, E5, E6 rotated %.2d%c, %.2d%c, %.2d%c in X, Y, and Z, and translated %.2dmm in X, %.2dmm in Y, %.2dmm in Z\n', round(rotx2), char(176), round(roty2), char(176), round(rotz2), char(176), round(t2(1)), round(t2(2)), round(t2(3)))


% Print distance info
E1_dist = E1(lastE1,:) - E1(1,:);
E1lineXY = sqrt((E1(lastE1,1)-E1(1,1))^2+(E1(lastE1,2)-E1(1,2))^2);
E1line = sqrt((E1(lastE1,1)-E1(1,1))^2+(E1(lastE1,2)-E1(1,2))^2+(E1(lastE1,3)-E1(1,3))^2);
fprintf('E1 distance moved %d time segments is X: %dmm, Y: %dmm, Z: %dmm OR XY distance: %dmm, XYZ distance: %dmm\n', lastE1, round(E1_dist(1)), round(E1_dist(2)), round(E1_dist(3)), round(E1lineXY), round(E1line))

E2_dist = E2(lastE2,:) - E2(1,:);
E2lineXY = sqrt((E2(lastE2,1)-E2(1,1))^2+(E2(lastE2,2)-E2(1,2))^2);
E2line = sqrt((E2(lastE2,1)-E2(1,1))^2+(E2(lastE2,2)-E2(1,2))^2+(E2(lastE2,3)-E2(1,3))^2);
fprintf('E2 distance moved %d time segments is X: %dmm, Y: %dmm, Z: %dmm OR XY distance: %dmm, XYZ distance: %dmm\n', lastE2, round(E2_dist(1)), round(E2_dist(2)), round(E2_dist(3)), round(E2lineXY), round(E2line))

E3_dist = E3(lastE3,:) - E3(1,:);
E3lineXY = sqrt((E3(lastE3,1)-E3(1,1))^2+(E3(lastE3,2)-E3(1,2))^2);
E3line = sqrt((E3(lastE3,1)-E3(1,1))^2+(E3(lastE3,2)-E3(1,2))^2+(E3(lastE3,3)-E3(1,3))^2);
fprintf('E3 distance moved %d time segments is X: %dmm, Y: %dmm, Z: %dmm OR XY distance: %dmm, XYZ distance: %dmm\n', lastE3, round(E3_dist(1)), round(E3_dist(2)), round(E3_dist(3)), round(E3lineXY), round(E3line))

E4_dist = E4(lastE4,:) - E4(1,:);
E4lineXY = sqrt((E4(lastE4,1)-E4(1,1))^2+(E4(lastE4,2)-E4(1,2))^2);
E4line = sqrt((E4(lastE4,1)-E4(1,1))^2+(E4(lastE4,2)-E4(1,2))^2+(E4(lastE4,3)-E4(1,3))^2);
fprintf('E4 distance moved %d time segments is X: %dmm, Y: %dmm, Z: %dmm OR XY distance: %dmm, XYZ distance: %dmm\n', lastE4, round(E4_dist(1)), round(E4_dist(2)), round(E4_dist(3)), round(E4lineXY), round(E4line))

E5_dist = E5(lastE5,:) - E5(1,:);
E5lineXY = sqrt((E5(lastE5,1)-E5(1,1))^2+(E5(lastE5,2)-E5(1,2))^2);
E5line = sqrt((E5(lastE5,1)-E5(1,1))^2+(E5(lastE5,2)-E5(1,2))^2+(E5(lastE5,3)-E5(1,3))^2);
fprintf('E5 distance moved %d time segments is X: %dmm, Y: %dmm, Z: %dmm OR XY distance: %dmm, XYZ distance: %dmm\n', lastE5, round(E5_dist(1)), round(E5_dist(2)), round(E5_dist(3)), round(E5lineXY), round(E5line))

E6_dist = E6(lastE6,:) - E6(1,:);
E6lineXY = sqrt((E6(lastE6,1)-E6(1,1))^2+(E6(lastE6,2)-E6(1,2))^2);
E6line = sqrt((E6(lastE6,1)-E6(1,1))^2+(E6(lastE6,2)-E6(1,2))^2+(E6(lastE6,3)-E6(1,3))^2);
fprintf('E6 distance moved %d time segments is X: %dmm, Y: %dmm, Z: %dmm OR XY distance: %dmm, XYZ distance: %dmm\n', lastE6, round(E6_dist(1)), round(E6_dist(2)), round(E6_dist(3)), round(E6lineXY), round(E6line))


function [rotx, roty, rotz, t] = tracking(E1, E2, E3, final_row)
    % Calculate rotation matrix and translation vector between first and last
    % frames
    first = [(E1(1,1) + E2(1,1) + E3(1,1))/3; (E1(1,2) + E2(1,2) + E3(1,2))/3; (E1(1,3) + E2(1,3) + E3(1,3))/3];
    last = [(E1(final_row,1) + E2(final_row,1) + E3(final_row,1))/3; (E1(final_row,2) + E2(final_row,2) + E3(final_row,2))/3; (E1(final_row,3) + E2(final_row,3) + E3(final_row,3))/3];
    
    % find mean column wise
    centroid_first = mean(first, 2);
    centroid_last = mean(last, 2);
    
    % subtract mean
    first_mu = first - repmat(centroid_first, 1, 3);
    last_mu = last - repmat(centroid_last, 1, 3);
    
    % calculate covariance matrix (is this the correct terminology?)
    %H = first_mu.' * last_mu;
    H = first_mu * last_mu';
    
    % find rotation
    [U,S,V] = svd(H);
    R = V*U';
    
    if det(R) < 0
         sprintf("det(R) < R, reflection detected!, correcting for it ...\n");
         V(:,3) = V(:,3) * -1;
         R = V*U';
    end
    R
    % Euler angles
    rotx = atan2d(R(2,3), R(3,3));
    roty = atan2d(-R(1,3), sqrt(R(2,3)^2+R(3,3)^2));
    rotz = atan2d(R(1,2), R(1,1));
    
    t = -R*centroid_first + centroid_last;
    %t = -R*(repmat(centroid_first, 3, 1)) + (repmat(centroid_last, 3, 1));
end

function [uhat, vhat, what, origin] = movement(E1, E2, E3, final_row)
    % Define c.s. in starting and end frames, then calculate rotation 
    % matrix and translation vector between first and last frames
    first_center = [(E1(1,1) + E2(1,1) + E3(1,1))/3; (E1(1,2) + E2(1,2) + E3(1,2))/3; (E1(1,3) + E2(1,3) + E3(1,3))/3];
    %last_center = [(E1(final_row,1) + E2(final_row,1) + E3(final_row,1))/3; (E1(final_row,2) + E2(final_row,2) + E3(final_row,2))/3; (E1(final_row,3) + E2(final_row,3) + E3(final_row,3))/3];

    firstE1 = [E1(1,1); E1(1,2); E1(1,3)];
    firstE2 = [E2(1,1); E2(1,2); E2(1,3)];
    firstE3 = [E3(1,1); E3(1,2); E3(1,3)];

    % Set E1 as the origin
    origin = firstE1;

    % Define plane from vectors extending from origin to two other points
    v12 = firstE2 - origin;
    v13 = firstE3 - origin;

    % Define first basis vector by normalizing v12 to create orthonormal
    % basis vector uhat
    uhat = v12/norm(v12);

    % Define second basis vector by using the Gram-Schmidt process to find
    % a vector perpendicular to uhat within the plane and normalize it to
    % get the second basis vector vhat
    % vperp = v13 - (dot(v13,uhat))*uhat;
    vperp = cross(uhat, v13);
    vhat = vperp / norm(vperp);

    % Calculate the third basis vector using the cross product
    what = cross(uhat, vhat);
end


% Specify circle vectors
function [center, radius, x, y, z] = triangle2circle(P1, P2, P3)
    % Describe the edges of the triangle by defining two vectors from P1
    u = P2 - P1;
    v = P3 - P1;

    % Define normal vector (orientation in 3D)
    norm_temp = cross(u,v);
    normal = norm_temp / norm(norm_temp);

    % Set up linear set of equations based off three points and solve it to
    % generate the circle center, then use that to calculate the radius
    A = [2*u; 2*v; normal];
    b = [dot(P2, P2) - dot(P1, P1); dot(P3, P3) - dot(P1, P1); dot(normal, P1)];
    center = (A \ b)'; % Solve the linear system
    radius = norm(center - P1);

    % 6 variables to parameterize 3D circle
    phi = atan2(normal(2),normal(1)); % azimuth angle, in [-pi, pi]
    theta = atan2(sqrt(normal(1)^2 + normal(2)^2), normal(3)); % zenith angle, in [0,pi]    
    t = 0:pi/32:2*pi;
    x = center(1) - radius*( cos(t)*sin(phi) + sin(t)*cos(theta)*cos(phi) );
    y = center(2) + radius*( cos(t)*cos(phi) - sin(t)*cos(theta)*sin(phi) );
    z = center(3) + radius*sin(t)*sin(theta);
    % plot3(x,y,z,'r')
    % patch(x,y,z,'r')
    % hold on
end

function plotDisk(x, y, z, color)
    plot3(x,y,z, 'Color', color)
    patch(x,y,z, color)
    hold on
end

function plotCurvature(v, run_video, start, stop, color, E1, E2, E3, M1, M2, M3, I1, I2, I3)
    % Plot overall distance + path traveled
    for i = start:100:stop
        % For module 1 since we have all points, we have to end at the starting node (1)
        [centerE1, radiusE1, xE1, yE1, zE1] = triangle2circle(E1(i,:), E2(i,:), E3(i,:));
        %hold on
        [centerM1, radiusM1, xM1, yM1, zM1] = triangle2circle(M1(i,:), M2(i,:), M3(i,:));
        %hold on
        [centerI1, radiusI1, xI1, yI1, zI1] = triangle2circle(I1(i,:), I2(i,:), I3(i,:));
        %hold on

        % Cute colors
        plotDisk(xE1, yE1, zE1, hex2rgb(color))
        plotDisk(xM1, yM1, zM1, hex2rgb(color))
        plotDisk(xI1, yI1, zI1, hex2rgb(color))
    
        % Tendons 1 - 6 since we're missing node 1 points
        Tendon1 = [E1(i,:); M1(i,:); I1(i,:)];
        plot3(Tendon1(:,1),Tendon1(:,2),Tendon1(:,3), 'Color',"#2E1503", 'LineWidth', 2, 'MarkerEdgeColor','k')
        %hold on
        Tendon2 = [E2(i,:); M2(i,:); I2(i,:)];
        plot3(Tendon2(:,1),Tendon2(:,2),Tendon2(:,3), 'Color',"#2E1503", 'LineWidth', 2, 'MarkerEdgeColor','k')
        %hold on
        Tendon3 = [E3(i,:); M3(i,:); I3(i,:)];
        plot3(Tendon3(:,1),Tendon3(:,2),Tendon3(:,3), 'Color',"#2E1503", 'LineWidth', 2, 'MarkerEdgeColor','k')
        %hold on
        if i == stop
            hold on
        end
        
        title("Timestep ", i);
        % Write video frame
        if run_video == 1
            view(45,0);
            frame = getframe(gcf);
            writeVideo(v,frame)
            hold off
        end
    end
end

function plotCurvatureFinal(i, color, E1, E2, E3, M1, M2, M3, I1, I2, I3)
        nexttile
        % For module 1 since we have all points, we have to end at the starting node (1)
        [centerE1, radiusE1, xE1, yE1, zE1] = triangle2circle(E1(i,:), E2(i,:), E3(i,:));
        hold on
        [centerM1, radiusM1, xM1, yM1, zM1] = triangle2circle(M1(i,:), M2(i,:), M3(i,:));
        hold on
        [centerI1, radiusI1, xI1, yI1, zI1] = triangle2circle(I1(i,:), I2(i,:), I3(i,:));
        hold on

        % Cute colors
        plotDisk(xE1, yE1, zE1, hex2rgb(color))
        plotDisk(xM1, yM1, zM1, hex2rgb(color))
        plotDisk(xI1, yI1, zI1, hex2rgb(color))
    
        % Tendons 1 - 6 since we're missing node 1 points
        Tendon1 = [E1(i,:); M1(i,:); I1(i,:)];
        plot3(Tendon1(:,1),Tendon1(:,2),Tendon1(:,3), 'Color',"#2E1503", 'LineWidth', 2, 'MarkerEdgeColor','k')
        hold on
        Tendon2 = [E2(i,:); M2(i,:); I2(i,:)];
        plot3(Tendon2(:,1),Tendon2(:,2),Tendon2(:,3), 'Color',"#2E1503", 'LineWidth', 2, 'MarkerEdgeColor','k')
        hold on
        Tendon3 = [E3(i,:); M3(i,:); I3(i,:)];
        plot3(Tendon3(:,1),Tendon3(:,2),Tendon3(:,3), 'Color',"#2E1503", 'LineWidth', 2, 'MarkerEdgeColor','k')
        hold on
        
        title("Maximum Curvature Depending on Motor Movement");
        view(90,0);
        axis image
        

end