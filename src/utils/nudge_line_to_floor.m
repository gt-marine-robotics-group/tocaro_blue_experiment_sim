function line_params = nudge_line_to_floor(line_params, range_m, floor_value)
%NUDGE_LINE_TO_FLOOR Ensure line s(r)=a+b*r satisfies floor at min/max range.

line_params = line_params(:);
if numel(line_params) ~= 2
    error("nudge_line_to_floor:BadLineParams", "Expected 2x1 [offset;slope].");
end

offset = line_params(1);
slope = line_params(2);

r_min = min(range_m);
r_max = max(range_m);

s_min = offset + slope * r_min;
s_max = offset + slope * r_max;

needed = max([floor_value - s_min, floor_value - s_max, 0]);
offset = offset + needed;

line_params = [offset; slope];
end
