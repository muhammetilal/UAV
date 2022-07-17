function Te_0 = sampleWGR(Te_w, Tw_0, bounds)
    sampleBound = rand(6, 1) .* (bounds(:, 2) - bounds(:, 1)) + bounds(:, 1);
    Tsample_w = trvec2tform(sampleBound(1:3)') * eul2tform(sampleBound(4: 6)', "XYZ");
    Te_0 = Tw_0 * Tsample_w * Te_w;
end