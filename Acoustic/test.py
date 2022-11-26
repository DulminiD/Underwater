
# import required module
from playsound import playsound
import time
import random
from mutagen.mp3 import MP3

# for playing note.wav file
audio = MP3("/home/mcpslab/Downloads/beep-01a.mp3")
print(audio.info.length)
output_name = 'transmitter_bits' + str(time.ctime())

randomlist = []
for i in range(0,64):
    n = random.randint(0,1)
    randomlist.append(n)
print(randomlist)
print(len(randomlist))

with open(output_name + ".txt", mode='w') as file:
    for i in randomlist:
        file.write(str(i))

for i in randomlist:
    if i == 1:
        playsound('/home/mcpslab/Downloads/beep-01a.mp3')
    else:
        time.sleep(0.55)
