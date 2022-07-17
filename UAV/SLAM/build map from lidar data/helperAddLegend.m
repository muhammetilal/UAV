function helperAddLegend(hAx, labels)

% Add a legend to the axes
hLegend = legend(hAx, labels{:});

% Set text color and font weight
hLegend.TextColor  = [1 1 1];
hLegend.FontWeight = 'bold';
end