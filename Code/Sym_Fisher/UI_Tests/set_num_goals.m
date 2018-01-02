function set_num_goals( source, events )
%UNTITLED11 Summary of this function goes here
%   Detailed explanation goes here

global ng;
val = source.Value;
maps = source.String;
fprintf('The number of goals is %d\n', str2double(maps{val}));
disp('In set num of goals. ');
ng = str2double(maps{val});
reset_sym_math;
set_goal_pos(source, events);
end

