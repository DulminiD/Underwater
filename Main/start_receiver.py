import threading
import sys
sys.path.insert(0, '/home/mcpslab/Desktop/Underwater-github/Underwater/LED/')
sys.path.insert(0, '/home/mcpslab/Desktop/Underwater-github/Underwater/Camera/')

from rgb import run_RGB
from mic import start_mic

def methodOne():
    print('Optical')
    run_RGB()

    
def methodTwo():
    print('Acoustic')
    start_mic()

if __name__ == "__main__":
    t1 = threading.Thread(target=methodOne())
    t2 = threading.Thread(target=methodTwo())
            
    t1.start()
    t2.start()
            
    t1.join()
    t2.join()
