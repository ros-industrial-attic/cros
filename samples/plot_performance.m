times={};
fd = fopen('../build/bin/times.txt','r');
ntime=0;
while(~feof(fd))
    ntime=ntime+1;
    tline = fgetl(fd);
    times{ntime} = sscanf(tline,'%f')';
end
fclose(fd)
clear(tline)
x_axis = 1:ntime;
mean_times = cellfun(@mean,times);
stddev_times = cellfun(@std,times);
%bar(x_axis, mean_times)
%hold on,
h = errorbar(x_axis, mean_times, stddev_times);
set(gca,'YScale','log')
%ylabel('\mus/msg')
c_axis=axis;
axis([0.9 ntime+0.1 c_axis(3) c_axis(4)])

figure
for n_hist=1:ntime
    subplot(ntime,1,n_hist), hist(times{n_hist}, length(times{n_hist})/5)
end