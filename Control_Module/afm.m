function [ element ] = afm( map, name, index )
%Accesses array elements inside map
    matrix = map(name);
    element = matrix(index{:});
end

