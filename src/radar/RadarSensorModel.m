classdef (Abstract) RadarSensorModel
    methods (Abstract)
        measurements = measure(obj, target_poses)
    end
end
