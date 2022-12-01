import threading
import sys
sys.path.insert(0, '/home/mcpslab/Desktop/Underwater-github/Underwater/LED/')
sys.path.insert(0, '/home/mcpslab/Desktop/Underwater-github/Underwater/Acoustic/')

from random_bits_final import run_stream
from test import sound

def methodOne():
    print('Optical')
    run_stream()

    
def methodTwo():
    print('Acoustic')
    sound()

if __name__ == "__main__":
    t1 = threading.Thread(target=methodOne())
    t2 = threading.Thread(target=methodTwo())
            
    t1.start()
    t2.start()
            
    t1.join()
    t2.join()
