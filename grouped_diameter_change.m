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

inchworm = readmatrix('iNCHWORM - FULL RESULTS .csv','Range','C6:BD3785');
final_row_inchworm = 3507; %3780;

hybrid_inchworm = readmatrix('HYRBID INCHWORM WITH SUPPORT MARKERS.csv','Range','C6:BG3512');
final_row_hybrid = 3507;

C = readmatrix('C.csv','Range','C6:BD4913');
final_row_C = 3507; %4908;

% Inchworm
[E1,E2,E3,E4,E5,E6,M1,M2,M3,M4,M5,M6,I1,I2,I3,I4,I5,I6] = partitionData(inchworm, final_row_inchworm);
[E1Diameter, E2Diameter, M1Diameter, M2Diameter, I1Diameter, I2Diameter,y] = calcDiameter(E1,E2,E3,E4,E5,E6,M1,M2,M3,M4,M5,M6,I1,I2,I3,I4,I5,I6,final_row_inchworm);

% Hybrid Inchworm
[E1,E2,E3,E4,E5,E6,M1,M2,M3,M4,M5,M6,I1,I2,I3,I4,I5,I6] = partitionData(hybrid_inchworm, final_row_hybrid);
[E1DiameterH, E2DiameterH, M1DiameterH, M2DiameterH, I1DiameterH, I2DiameterH,yH] = calcDiameter(E1,E2,E3,E4,E5,E6,M1,M2,M3,M4,M5,M6,I1,I2,I3,I4,I5,I6,final_row_hybrid);

% C
[E1,E2,E3,E4,E5,E6,M1,M2,M3,M4,M5,M6,I1,I2,I3,I4,I5,I6] = partitionData(C, final_row_C);
[E1DiameterC, E2DiameterC, M1DiameterC, M2DiameterC, I1DiameterC, I2DiameterC,yC] = calcDiameter(E1,E2,E3,E4,E5,E6,M1,M2,M3,M4,M5,M6,I1,I2,I3,I4,I5,I6,final_row_C);




inchwormData = [E1Diameter; E2Diameter; M1Diameter; M2Diameter; I1Diameter; I2Diameter]; %Diameter
hybridInchwormData = [E1DiameterH; E2DiameterH; M1DiameterH; M2DiameterH; I1DiameterH; I2DiameterH];
CData = [E1DiameterC; E2DiameterC; M1DiameterC; M2DiameterC; I1DiameterC; I2DiameterC];

E1Diameter  = E1Diameter(:);
E2Diameter  = E2Diameter(:);
M1Diameter  = M1Diameter(:);
M2Diameter  = M2Diameter(:);
I1Diameter  = I1Diameter(:);
I2Diameter  = I2Diameter(:);

E1DiameterH  = E1DiameterH(:);
E2DiameterH  = E2DiameterH(:);
M1DiameterH  = M1DiameterH(:);
M2DiameterH  = M2DiameterH(:);
I1DiameterH  = I1DiameterH(:);
I2DiameterH  = I2DiameterH(:);

E1DiameterC  = E1DiameterC(:);
E2DiameterC  = E2DiameterC(:);
M1DiameterC  = M1DiameterC(:);
M2DiameterC  = M2DiameterC(:);
I1DiameterC  = I1DiameterC(:);
I2DiameterC  = I2DiameterC(:);

diamGroups = { ...
    E1Diameter, E2Diameter, M1Diameter, M2Diameter, I1Diameter, I2Diameter, ...
    E1DiameterH, E2DiameterH, M1DiameterH, M2DiameterH, I1DiameterH, I2DiameterH, ...
    E1DiameterC, E2DiameterC, M1DiameterC, M2DiameterC, I1DiameterC, I2DiameterC ...
};
diamGroups = cellfun(@(x) x(~isnan(x)), diamGroups, 'UniformOutput', false);

facesList = ["E1","E2","M1","M2","I1","I2"];
gaitList  = ["Inchworm", "Hybrid Inchworm", "Hybrid Sidewinding"];
allData = [];
faces   = [];
gait    = [];

for g = 1:3
    for f = 1:6
        idx = (g-1)*6 + f;  % index into diamGroups
        d = diamGroups{idx};
        allData = [allData; d];
        faces   = [faces; repmat(facesList(f), length(d),1)];
        gait    = [gait;  repmat(gaitList(g), length(d),1)];
    end
end

validIdx = allData ~= 0 & ~isnan(allData);  % keep only non-zero, non-NaN entries
allData = allData(validIdx);
faces   = faces(validIdx);
gait    = gait(validIdx);

diameterData = table(allData, categorical(faces), categorical(gait),'VariableNames', {'allData','faces','gait'});
figure

h = boxchart(diameterData.gait, diameterData.allData, 'GroupByColor', diameterData.faces);
ylabel('Diameter (mm)', 'FontSize', 20)

% Define your hex colors for the 6 faces
faceColorsHex = ["#C7A491","#EECFCA","#997B66","#919682","#C7CDBF","#595E48"];
faceColorsRGB = hex2rgb(faceColorsHex);   % convert hex to RGB
colororder(faceColorsRGB)

legend(categories(diameterData.faces))
title('Face Diameter Change Across Gaits','FontSize',20)
ylim([150 300])

% inchwormData = [E1Diameter'; E2Diameter'; M1Diameter'; M2Diameter'; I1Diameter'; I2Diameter']; %Diameter
% hybridInchwormData = [E1DiameterH'; E2DiameterH'; M1DiameterH'; M2DiameterH'; I1DiameterH'; I2DiameterH'];
% CData = [E1DiameterC'; E2DiameterC'; M1DiameterC'; M2DiameterC'; I1DiameterC'; I2DiameterC'];
% allData = [inchwormData; hybridInchwormData; CData]; % Diameter
% 
% InchFacestr = strings(size(allData)); % Face
% Gaitstr = strings(size(allData)); % Gait
% for i = 1:1:size(allData,1)
%     if (i > 0 && i < 3508) || (i > 21042 && i < 24550) || (i > 42084 && i < 46312)
%         InchFacestr(i) = "E1"; % Assigning the string for the inchworm data
%     elseif (i > 3507 && i < 7015)  || (i > 24549 && i < 28057) || (i > 46312 && i < 49819)
%         InchFacestr(i) = "E2"; % Assigning the string for the inchworm data
%     elseif (i > 7014 && i < 10522)  || (i > 28056 && i < 31564) || (i > 49819 && i < 53326)
%         InchFacestr(i) = "M1"; % Assigning the string for the inchworm data
%     elseif (i > 10521 && i < 14029)  || (i > 31563 && i < 35071) || (i > 53326 && i < 56833)
%         InchFacestr(i) = "M2"; % Assigning the string for the inchworm data
%     elseif (i > 14028 && i < 17536)  || (i > 35070 && i < 38578) || (i > 56833 && i < 60340)
%         InchFacestr(i) = "I1"; % Assigning the string for the inchworm data
%     elseif (i > 17535 && i < 21043)  || (i > 38577 && i < 42085) || (i > 60340 && i < 63848)
%         InchFacestr(i) = "I2"; % Assigning the string for the inchworm data
%     end
% 
%     if i > 0 && i < 21043
%         Gaitstr(i) = "Inchworm";
%     elseif i > 21042 && i < 42086
%         Gaitstr(i) = "Hybrid Inchworm";
%     elseif i > 42089 && i < 63848
%         Gaitstr(i) = "Hybrid Sidewinding";
%     end
% 
% end
% 
% diameterData = table(allData, InchFacestr, Gaitstr);
% diameterData.Properties
% 
% GaitOrder = {'Inchworm', 'Hybrid Inchworm', 'Hybrid Sidewinding'};
% diameterData.Gaitstr = categorical(diameterData.Gaitstr, GaitOrder);
% 
% boxchart(diameterData.Gaitstr, diameterData.allData, 'GroupByColor', diameterData.InchFacestr)
% colororder([hex2rgb("#C7A491");hex2rgb("#EECFCA");hex2rgb("#997B66");hex2rgb("#919682");hex2rgb("#C7CDBF");hex2rgb("#595E48");]);
% % boxplot(diameterData.allData)
% ylabel('Diameter (mm)')
% legend





figure(2)

scatter(E1Diameter, y, 'MarkerFaceColor', '#C7A491', 'MarkerEdgeColor', '#C7A491')
hold on
scatter(M1Diameter, y, 'MarkerFaceColor', '#EECFCA', 'MarkerEdgeColor', '#EECFCA')
hold on
scatter(I1Diameter, y, 'MarkerFaceColor', '#997B66', 'MarkerEdgeColor', '#997B66')
hold on
scatter(E2Diameter, y, 'MarkerFaceColor', '#919682', 'MarkerEdgeColor', '#919682')
hold on
scatter(M2Diameter, y, 'MarkerFaceColor', '#C7CDBF', 'MarkerEdgeColor', '#C7CDBF')
hold on
scatter(I2Diameter, y, 'MarkerFaceColor', '#595E48', 'MarkerEdgeColor', '#595E48')

xlim([175 300])
legend({'1st End', '1st Mid', '1st Inner', '2nd End', '2nd Mid', '2nd Inner'},'FontSize',20)
xlabel('Diameter (mm)', 'FontSize', 20)
ylabel('Timesteps', 'FontSize', 20)
title('Face Diameter Change Across Inchworm Gaits', 'FontSize',20)


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

function [E1,E2,E3,E4,E5,E6,M1,M2,M3,M4,M5,M6,I1,I2,I3,I4,I5,I6] = partitionData(M, final_row)

    % Gather all motor endpoints
    E1 = M(1:final_row, 1:3);
    E2 = M(1:final_row, 10:12);
    E3 = M(1:final_row, 19:21);
    E4 = M(1:final_row, 28:30);
    E5 = M(1:final_row, 37:39);
    E6 = M(1:final_row, 46:48);
    
    % Gather all midpoints
    M1 = M(1:final_row, 4:6);
    M2 = M(1:final_row, 13:15);
    M3 = M(1:final_row, 22:24);
    M4 = M(1:final_row, 31:33);
    M5 = M(1:final_row, 40:42);
    M6 = M(1:final_row, 49:51);
    
    % Gather all inner points
    I1 = M(1:final_row, 7:9);
    I2 = M(1:final_row, 16:18);
    I3 = M(1:final_row, 25:27);
    I4 = M(1:final_row, 34:36);
    I5 = M(1:final_row, 43:45);
    I6 = M(1:final_row, 52:54);

end

function [E1Diameter, E2Diameter, M1Diameter, M2Diameter, I1Diameter, I2Diameter,y] = calcDiameter(E1,E2,E3,E4,E5,E6,M1,M2,M3,M4,M5,M6,I1,I2,I3,I4,I5,I6,final_row)
    for i = 1:10:final_row
        % Time data
        y(i) = i;
    
        % Calculate disk face for each of the six icosahedrons
        [centerE1, radiusE1, xE1, yE1, zE1] = triangle2circle(E1(i,:), E2(i,:), E3(i,:));
        E1Diameter(i) = radiusE1*2;
    
        [centerM1, radiusM1, xM1, yM1, zM1] = triangle2circle(M1(i,:), M2(i,:), M3(i,:));
        M1Diameter(i) = radiusM1*2;
    
        [centerI1, radiusI1, xI1, yI1, zI1] = triangle2circle(I1(i,:), I2(i,:), I3(i,:));
        I1Diameter(i) = radiusI1*2;
    
        [centerE2, radiusE2, xE2, yE2, zE2] = triangle2circle(E4(i,:), E5(i,:), E6(i,:));
        E2Diameter(i) = radiusE2*2;
    
        [centerM2, radiusM2, xM2, yM2, zM2] = triangle2circle(M4(i,:), M5(i,:), M6(i,:));
        M2Diameter(i) = radiusM2*2;
    
        [centerI2, radiusI2, xI2, yI2, zI2] = triangle2circle(I4(i,:), I5(i,:), I6(i,:));
        I2Diameter(i) = radiusI2*2;
    end
end