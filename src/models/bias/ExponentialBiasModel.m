classdef ExponentialBiasModel < MixedBiasModel
    %ExponentialBiasModel Exponential bias in predicted range for both channels.

    methods
        function obj = ExponentialBiasModel()
            obj@MixedBiasModel(ExponentialBiasFunction(), ExponentialBiasFunction());
        end
    end
end
