% Lauren Ervin
% 2/23/26
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

I = imread("apriltag_test.png");
tagFamily = ["tag25h9","tag36h11"];

[id,loc,detectedFamily] = readAprilTag(I,tagFamily);

for idx = 1:length(id)
        % Display the ID and tag family
        disp("Detected Tag ID, Family: " + id(idx) + ", " ...
            + detectedFamily(idx));
 
        % Insert markers to indicate the locations
        markerRadius = 12;
        numCorners = size(loc,1);
        markerPosition = [loc(:,:,idx),repmat(markerRadius,numCorners,1)];
        I = insertShape(I,"FilledCircle",markerPosition,ShapeColor="red",Opacity=1);
end