times={};
fd = fopen('../build/bin/times.txt','r');
n_measure_expers=0;
while(~feof(fd))
    n_measure_expers=n_measure_expers+1;
    tline = fgetl(fd);
    times{n_measure_expers} = sscanf(tline,'%f')';
end
fclose(fd)
clear(tline)

diff_times = cellfun(@diff, times, 'UniformOutput', false);

x_axis = 1:n_measure_expers;
mean_times = cellfun(@mean, diff_times);
stddev_times = cellfun(@std, diff_times);
%bar(x_axis, mean_times)
%hold on,
h = errorbar(x_axis, mean_times, stddev_times);
set(gca,'YScale','log')
%ylabel('\mus/msg')
c_axis=axis;
axis([0.9 n_measure_expers+0.1 c_axis(3) c_axis(4)])

figure
for n_hist=1:n_measure_expers
    subplot(n_measure_expers,1,n_hist), hist(diff_times{n_hist}, length(diff_times{n_hist})/5)
end