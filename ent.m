function out = ent(p)
    
temp = find(p>0);
out = -sum(p(temp).*log2(p(temp)));

end