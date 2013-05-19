filelist='Body*.stl';
listing = dir(filelist);

for k=1:length(listing)
    if listing(k).isdir
        continue
    end
    reduceMeshBySlices(listing(k).name,'Z',10,1,2);
end
        