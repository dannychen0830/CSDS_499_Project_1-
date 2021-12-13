% sample from discrete distribution
function x = sample_state(p)
cdf = cumsum(p);
xi = rand;
for i=1:length(p)
    if cdf(i) > xi
        x = i;
        break
    end
end
end
