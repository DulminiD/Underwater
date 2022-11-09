
# import required module
from playsound import playsound
import time

# for playing note.wav file

for i in range (10):
    playsound('/home/mcpslab/Downloads/beep-01a.mp3')
    print('playing sound using  playsound')
    time.sleep(1/25)