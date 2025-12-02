% Lauren Ervin
% 12/1/25
% Tensegrity snake Vicon plotting script
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

M = readmatrix('UNDULATION VICON NEW.csv','Range','C6:BV14798');
final_row = 14793;

% Set last row that each node contains Vicon data
lastE1 = 13941;
lastE2 = 9280;
lastE3 = 14793;
lastE4 = 11390;
lastE5 = 1130;
lastE6 = 13940;

% Generate video info
% video_title = "testy10.mp4"; %Update with descriptive title and make sure not to overwrite previous video
% v = VideoWriter(video_title, "MPEG-4");
% v.FrameRate = 15; %Set video frame rate
% open(v);
% run_video = 1;
run_video = 0;

% Gather all motor endpoints
E1 = M(1:final_row, 10:12);
E2 = M(1:final_row, 46:48);
E3 = M(1:final_row, 1:3);
E4 = M(1:final_row, 19:21);
E5 = M(1:final_row, 28:30);
E6 = M(1:final_row, 37:39);

% Gather all midpoints
M1 = M(1:final_row, 13:15);
M2 = M(1:final_row, 49:51);
M3 = M(1:final_row, 4:6);
M4 = M(1:final_row, 22:24);
M5 = M(1:final_row, 31:33);
M6 = M(1:final_row, 40:42);

% Gather all inner points
I1 = M(1:final_row, 16:18);
I2 = M(1:final_row, 52:54);
I3 = M(1:final_row, 7:9);
I4 = M(1:final_row, 25:27);
I5 = M(1:final_row, 34:36);
I6 = M(1:final_row, 43:45);

% Plot overall distance + path traveled
for i = 1:100:final_row
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
    
    title("Timestep ", i);
    % Write video frame
    if run_video == 1
        frame = getframe(gcf);
        writeVideo(v,frame)
        hold off
    end
end

if run_video == 1
    close(v)
end


% Plot different gaits within movement
% Manually set time that different gaits occur in a movement sequence
figure(2)
t = tiledlayout(2,2);

for i = [1, 1650, 2700, 3800]
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
    elseif i == 1650 || i == 3800
        title('State 1');
    else
        title('State 2');
    end
end
title(t, 'Undulation Gait');




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