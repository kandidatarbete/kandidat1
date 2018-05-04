function exitangle_new = angleConverter(entryangle,exitangle)
if abs(abs(entryangle-exitangle)-pi) < 1e-2
    exitangle_new=entryangle;
else
    if exitangle == 0
        if entryangle > 0 && entryangle<pi
            exitangle_new=entryangle-pi/2;
            return;
        else
            exitangle_new=entryangle+pi/2;
            return;
        end
    end
    if entryangle == 0
       if exitangle > 0 && exitangle<pi
            exitangle_new=entryangle+pi/2;
            return;
        else
            exitangle_new =entryangle-pi/2;
            return;
        end
    end
    if exitangle-entryangle >0
        exitangle_new=entryangle + pi/2;
    else
        exitangle_new=entryangle-pi/2;
    end
    
end