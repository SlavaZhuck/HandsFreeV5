Fd = 16000;

filename_mic = sprintf('mic_%d.wav', Fd);
filename_filt = sprintf('filt_%d.wav', Fd);
filename_ble = sprintf('ble_%d.wav', Fd);

audiowrite(filename_mic,simout_mic,Fd);
audiowrite(filename_filt,simout_filt,Fd);
%audiowrite(filename_ble,simout_ble,Fd);