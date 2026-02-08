classdef ExponentialVarianceModel < MixedVarianceModel
    %ExponentialVarianceModel Exponential sigma variance for both channels.

    methods
        function obj = ExponentialVarianceModel()
            obj@MixedVarianceModel(ExponentialVarianceFunction(), ExponentialVarianceFunction());
        end
    end
end
