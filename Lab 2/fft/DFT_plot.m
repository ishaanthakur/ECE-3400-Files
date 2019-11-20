FFT_N = 256;

low_bin = 0;
high_bin = 9187;
bin_width = (high_bin - low_bin) / (FFT_N / 2 - 1);
bins = low_bin : bin_width : high_bin;

amplitudes = zeros(1, FFT_N / 2);

dft_plot = plot(bins, amplitudes);
xlim([0 5000]);
ylim([0 7000]);
xlabel('Frequency (Hz)');
ylabel('Amplitude');
dft_plot.XDataSource = 'bins';
dft_plot.YDataSource = 'amplitudes';

myserialport = serial('/dev/tty.usbmodem14201', 'BaudRate', 115200);
fopen(myserialport);

while 1
    amplitudes = fread(myserialport, FFT_N / 2, 'int16');
    refreshdata;
    drawnow;
end

fclose(myserialport);