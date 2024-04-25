# chapter 28 in python

# sudo apt-get install python3-pip
# python3 -m pip install pyserial
# sudo apt-get install python3-matplotlib

import matplotlib.pyplot as plt
import serial
import re
import genref
ser = serial.Serial('/dev/tty.usbserial-140',230400)
print('Opening port: ')
print(ser.name)

def read_plot_matrix():
    print('reading data length')
    # n_int is data length (100 for K), variable for others
    n_int = int(ser.read_until(b'\r\n')) # get N
    print('reading data length done')
    ref = []
    measured = []
    data_received = 0
    print("it is before while")
    print(n_int)
    while data_received < n_int:
        dat_str = ser.read_until(b'\n')  # get the data as a string, ints seperated by spaces
        data_text = str(dat_str,'utf-8')
        data = list(map(float,data_text.split()))

        if(len(data)==3):
            data_received += 1
            measured.append(data[1])
            ref.append(data[2])
            
            
    meanzip = zip(ref, measured)
    meanlist = []
    for i,j in meanzip:
        meanlist.append(abs(i-j))
    
    time_per_sample_ms = 0.2  # 0.2 ms per sample at 5000 samples per second
    t = [i * time_per_sample_ms for i in range(len(ref))]

    score = sum(meanlist)/n_int
    plt.title('Score = ' + str(score))
    # t = range(len(measured)) # time array
    plt.plot(t, measured,'r*-', label="Measured Angle") 
    plt.plot(t, ref,'b*-',label="Reference Angle")
    plt.ylabel('Angle')
    plt.xlabel('Time [ms]')
    plt.legend()
    plt.show()

def plot_traj(traj):
    t = range(len(ref))
    plt.plot(t,ref,'r*-')
    plt.ylabel('ange in degrees')
    plt.xlabel('index')
    plt.show()

has_quit = False
# menu loop
while not has_quit:
    print('PIC32 MOTOR DRIVER INTERFACE')
    # display the menu options; this list will grow
    print('\t\nb: Read Current Sensor \t\nc: Read Encoder Counts \t\nd: Read Encoder(Deg) \t\ne: Reset the Encoder \t\nf: Set PWM(-100 to 100) \t\ng: Set Current Gains \t\nh: Get Current Gains \t\ni: Set Position Gains \t\nj: Get Position Gains \t\nk: ITEST -- Test Current Control \t\nl: Go to Angle (Deg) \t\nm: Load Step Trajectory \t\nn: Load Cubic Trajectory \t\no: Execute Trajectory. \t\np: Unpower the Motor \t\nr: Get Mode\t\nz: Dummy Command \t\nq: Quit') # '\t' is a tab
    # read the user's choice
    selection = input('\nENTER COMMAND: ')
    selection_endline = selection+'\n'
     
    # send the command to the PIC32
    ser.write(selection_endline.encode()); # .encode() turns the string into a char array
    
    # take the appropriate action
    # there is no switch() in python, using if elif instead
    if (selection == 'z'):
        # example operation
        n_str = input('Enter number: ') # get the number to send
        n_int = int(n_str) # turn it into an int
        print('number = ' + str(n_int)) # print it to the screen to double check

        ser.write((str(n_int)+'\n').encode()); # send the number
        n_str = ser.read_until(b'\n');  # get the incremented number back
        n_int = int(n_str) # turn it into an int
        print('Got back: ' + str(n_int) + '\n') # print it to the screen

    elif selection == "b":
        # gets cur_current
        cur_current = ser.read_until(b'\r\n').decode()
        print(cur_current)
        cur_current_float = float(cur_current)
        print("Current [mA]: ", cur_current_float)
    elif selection == "c":
        # read encoder counts
        enc_cnts_str = ser.read_until(b'\n')
        enc_cnts_int = int(enc_cnts_str)
        print("Encoder count: ", enc_cnts_int)
    elif selection == "d":
        # read encoder degree
        enc_cnts_str = ser.read_until(b'\r\n')
        enc_cnts_int = float(enc_cnts_str)
        print("Encoder count (deg): ", enc_cnts_int)
    elif selection == "e":
        # reset encoder
        print("Encoder count reset to 0")
    

    elif (selection == "f"):
        # drive PWM @ DutyCycle + direc
        pwm_str = input('Enter PWM [-100, 100]: ') # get the number to send
        pwm_int = int(pwm_str)
        ser.write((str(pwm_int)+'\n').encode()) # send the PWM
        print("Should see motor spinning in direc at PWM!")

    elif (selection == "g"):
        kp_str = input('Enter Kp value:')
        kp = float(kp_str)
        ser.write((str(kp) +'\n').encode())
        ki_str = input('Enter Ki value:')
        ki = float(ki_str)
        ser.write((str(ki) +'\n').encode())
        print(f"Selected kp: {kp}, ki: {ki}")

    elif (selection == "h"):

        cur_gains_kpki = ser.read_until(b'\r\n').decode().split(' ')
        kp, ki = float(cur_gains_kpki[0]), float(cur_gains_kpki[1])

        # cur_gains_kpki = ser.read_until(b'\n').decode().split(';')
        # float_pattern = r"[-+]?\d*\.\d+|[-+]?\d+"

        # for gain in cur_gains_kpki:
        #     if 'Kp' in gain:
        #         matches = re.search(float_pattern, gain)
        #         if matches:
        #             kp = float(matches.group())
        #     elif 'Ki' in gain:
        #         matches = re.search(float_pattern, gain)
        #         if matches:
        #             ki = float(matches.group())

        print(f"Current gains Kp: {kp}, Ki: {ki}")
        

    elif (selection == "k"):
        print("Running ITEST!")
        # read the data back as int in reverse order
        read_samples = 100
        measured = []
        ref = []
        try:
            while read_samples > 1:
                data_read = ser.read_until(b'\n',50)
                data_text = str(data_read,'utf-8')
                data = list(map(int,data_text.split()))

                if(len(data)==3):
                    read_samples = data[0]
                    measured.append(data[1])
                    ref.append(data[2])
            print("Read Done")
        except ValueError:
            pass
        
        # plot it
        t = range(len(measured)) # time array
        plt.plot(t, measured,'r*-', label="Measured Current [mA]")    # this is in range 1024. Need to map it to PR scale.
        plt.plot(t, ref,'b*-',label="Reference Current [mA]")
        plt.ylabel('value')
        plt.xlabel('sample')
        plt.legend()
        plt.show()
    elif selection == "l":
        # go to angle (deg)
        print("HOLDing!")
        # set position gains (kp, ki)
        angle = input('Enter desired angle to hold: ')
        ser.write((angle+'\n').encode()) # send the kp
            
    elif selection == "m":
        # load step trajectory
        ref = genref.genRef('step')
        if len(ref) > 3000:
            print('too many, ref posn only 3000 size')
        else:
            plot_traj(ref)
        # send: number of data points, each data point
        ser.write((str(len(ref))+'\n').encode())
        for i in ref:
            ser.write((str(i)+'\n').encode())

    elif selection == "n":
        # load cubic trajectory
        ref = genref.genRef('cubic')
        if len(ref) > 3000:
            print('too many, ref posn only 3000 size')
        else:
            plot_traj(ref)
                # send: number of data points, each data point
        ser.write((str(len(ref))+'\n').encode())
        for i in ref:
            ser.write((str(i)+'\n').encode())
    
    elif selection == "o":
        # execute trajectory
        print("Execute trajectory")
        print("start function read_plot_matrix")
        read_plot_matrix()
        print("after read_plot_matrix")

    elif selection == "p":
        # unpower motor (go to IDLE)
        print("Motor Unpowered to IDLE")

    
    elif (selection == 'q'):
        print('Exiting client')
        has_quit = True; # exit client
        # be sure to close the port
        ser.close()

    elif selection == "r":
        # read what mode
        input_string = ser.read_until(b'\n').decode()
        print(input_string)
        match = re.search(r'[A-Z]+(?=\s*$)', input_string)

        # Check if a match was found and print the result
        if match:
            last_word = match.group()
            print(last_word)
        else:
            print("No match found")
        print("Mode: ", last_word)

    elif selection == "i":
        # set position gains (kp, ki)
        kp = input('Enter position kp [float]: ')
        ser.write((kp+'\n').encode()) # send the kp
        ki = input('Enter position ki [float]: ')
        ser.write((ki+'\n').encode()) # send the ki
        kd = input('Enter position kd [float]: ')
        ser.write((kd+'\n').encode()) # send the kd
        print(f"Selected kp: {kp}, ki: {ki}, kd: {kd}")

    elif selection == "j":
        # get position gains
        cur_gains_kpki = ser.read_until(b'\n').decode().split(' ')
        kp, ki, kd = float(float(cur_gains_kpki[0])), float(cur_gains_kpki[1]), float(cur_gains_kpki[2])
        print(f"Current kp: {kp}, ki: {ki}, kd: {kd}")

    elif selection == "l":
        # go to angle (deg)
        print("HOLDing!")
        # set position gains (kp, ki)
        angle = input('Enter desired angle to hold: ')
        ser.write((angle+'\n').encode()) # send the kp
    

    else:
        print('Invalid Selection ' + selection_endline)



