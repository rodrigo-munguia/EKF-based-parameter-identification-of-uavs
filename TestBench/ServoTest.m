clear all
% port at which your arduino is connected

if ~isempty(instrfind)
     fclose(instrfind);
      delete(instrfind);
end


port = '/dev/ttyS99';

% model of your arduino board

board = 'Uno';

% creating arduino object with servo library

arduino_board = arduino(port, board, 'Libraries', 'Servo');

% creating servo motor object

%servo_motor = servo(arduino_board, 'D8','MinPulseDuration',1*10^-3,'MaxPulseDuration',2*10^-3);
servo_motor = servo(arduino_board, 'D8');


% loop to rotate servo motor from 0 to 180

for angle = .2:0.2:.8

   writePosition(servo_motor, angle);

   current_position = readPosition(servo_motor);

   current_position = current_position * 180;   

   % print current position of servo motor

   fprintf('Current position is %d\n', current_position);   

   % small delay is required so that servo can be positioned at the

   % angle told to it.

   pause(2);

end

% bring back motor to 0 angle position

writePosition(servo_motor, .2);