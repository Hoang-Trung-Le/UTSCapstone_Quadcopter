function map = ReadProperty(document)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here

str = extractFileText(document);
l = 0.1173;
m = double(extractBetween(str, "Mass = ", " kilograms"));
Ixx = double(extractBetween(str, "Px = ", lettersPattern));
Iyy = double(extractBetween(str, "Py = ", lettersPattern));
Izz = double(extractBetween(str, "Pz = ", lettersPattern));

keySet = {'mass', 'armLength', 'Ixx', 'Iyy', 'Izz'};
valueSet = [m, l, Ixx, Iyy, Izz];
map = containers.Map(keySet, valueSet);

end

