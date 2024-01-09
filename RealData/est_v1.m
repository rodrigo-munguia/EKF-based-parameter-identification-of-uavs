clear all



filename = '..\\..\\LOGS\\19-05-02_20-10-34.bin';

log = Ardupilog(filename);

%log_filtered = log.filterMsgs({'IMU'})

sliced_log = log.getSlice([708, 870], 'TimeS');

log_struct = sliced_log.getStruct()