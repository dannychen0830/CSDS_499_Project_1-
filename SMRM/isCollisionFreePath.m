function b = isCollisionFreePath(path,bw)
b = 1;
for i=1:length(path)
    if ~isCollisionFree(path(:,i),bw)
        b = 0;
        break
    end
end