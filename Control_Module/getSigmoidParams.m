function params = getSigmoidParams(limits)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
    params = [6/(limits(2) - limits(1)) mean(limits)];
end

