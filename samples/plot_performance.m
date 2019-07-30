times={};
fd = fopen('../build/bin/times.txt','r');
ntime=1;
while(~feof(fd))
    tline = fgetl(fd);
    times{ntime} = sscanf(tline,'%f')';
    ntime=ntime+1;
end
fclose(fd)
clear(tline)
x_axis = 1:(ntime-1);
mean_times = cellfun(@mean,times);
stddev_times = cellfun(@std,times);
%bar(x_axis, mean_times)
%hold on,
errorbar(mean_times, stddev_times)
