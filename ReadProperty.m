function map = ReadProperty(document)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here

str = extractFileText(document);
mass = double(extractBetween(str, "Mass = ", " kilograms"));
Ixx = double(extractBetween(str, "Px = ", " "));
Iyy = double(extractBetween(str, "Py = ", " "));
Izz = double(extractBetween(str, "Pz = ", lettersPattern));
if Ixx ~= Iyy
    Iyy = Ixx;
end

keySet = {'mass', 'Ixx', 'Iyy', 'Izz'};
valueSet = [mass, Ixx, Iyy, Izz];
map = containers.Map(keySet, valueSet);

end

