function bin_edges = approx_quantile_edges(sample_values, num_bins)
%APPROX_QUANTILE_EDGES Approximate quantile bin edges without toolboxes.

sample_values = sample_values(:);
num_samples = numel(sample_values);

if num_samples < 2
    bin_edges = [-Inf; Inf];
    return;
end

sorted_values = sort(sample_values);
edge_indices = round(linspace(1, num_samples, num_bins + 1));
edge_indices = max(min(edge_indices, num_samples), 1);

bin_edges = sorted_values(edge_indices);

for edge_index = 2:numel(bin_edges)
    if bin_edges(edge_index) <= bin_edges(edge_index - 1)
        bin_edges(edge_index) = bin_edges(edge_index - 1) + eps(bin_edges(edge_index - 1) + 1.0);
    end
end

bin_edges = bin_edges(:);
end
