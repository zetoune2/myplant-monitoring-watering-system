##############
## Script listens to serial port and writes contents into a file
##
##Copyright Zetoune Corp 2020
##
##############

import serial
import logging

from logging.handlers import RotatingFileHandler

#VARIABLE DEFINITION
serial_port = '/dev/ttyACM0'
baud_rate = 9600;
log_path ="/home/zetoune/LOG_MYPLANTS/myWTFBox.log"

logger = logging.getLogger()
# on met le niveau du logger a DEBUG, comme ca il ecrit tout
logger.setLevel(logging.DEBUG)

formatter = logging.Formatter('%(asctime)s :: %(levelname)s :: %(message)s')


# creation dun handler qui va rediriger une ecriture du log vers
# un fichier en mode 'append', avec 1 backup et une taille max de 1Mo
file_handler = RotatingFileHandler(log_path, 'a', 3500000, 10)

formatter = logging.Formatter('%(asctime)s :: %(levelname)s :: %(message)s')

# on lui met le niveau sur DEBUG, on lui dit qu'il doit utiliser le formateur
# cree precedement et on ajoute ce handler au logger
file_handler.setLevel(logging.DEBUG)
file_handler.setFormatter(formatter)
logger.addHandler(file_handler)

# creation dun second handler qui va rediriger chaque ecriture de log
# sur la console
stream_handler = logging.StreamHandler()
stream_handler.setLevel(logging.DEBUG)
logger.addHandler(stream_handler)

serial_output = serial.Serial(serial_port, baud_rate)

while True:
    line = serial_output.readline();
    logger.info(line);
