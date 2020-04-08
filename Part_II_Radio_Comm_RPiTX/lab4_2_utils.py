import shlex, subprocess
import numpy as np
import matplotlib.pyplot as plt
import pyaudio, threading,time, sys, threading,time, serial
import queue as Queue
from numpy import pi, sin, zeros, r_
from scipy import signal
from rtlsdr import RtlSdr
import sounddevice as sd


class Radio:
    """a radio class"""
    
    freq_rx = 146430.0 # in KHz
    freq_tx = 146430.0  # in KHz
    
    ptx =[] #process for TX
    prx =[] #process for RX
    
    prled=[] # process for red LED toggel
    pgled=[] # process for green LED toggel
    
    istx = False
    isrx = False
    
    #parameters for rtl_sdr
    sdr_gain = 0
    sdr_squelch = 0
    sdr_ppm = 0
    
    #virtual receiver audio gain for rtl_fm. 
    audio_rcv_gain = 1.0
    
    #default audio devices
    audiodev_in = "plughw:CARD=Loopback,DEV=0"
    audiodev_out = "plughw:CARD=Loopback,DEV=0"
    
    def __init__(self, freq_rx=146430.0, freq_tx=146430.0):
        self.freq_rx = freq_rx;
        self.freq_tx = freq_tx;
    
    
    # function generates a pure tone at a given frequency. 
    def tune(self):    
        if self.istx:
            print ("transmitter seems to be transmitting already, try stopping first")
            return
        
        try:
            # run tune process
            command_line = "sudo tune -f %.3f" %self.freq_tx;
            print(command_line)
            
            args = shlex.split(command_line)
            self.ptx = subprocess.Popen(args)
            print(command_line)
            
            self.istx = True
        except:
            print("Something bad happend trying to transmit a tone")
    
    # function transmits Narrowband FM streamed from audiodev_in. 
    def NFMtx(self):
        
        if self.istx:
            print ("transmitter seems to be transmitting already, try stopping first")
            return
        
        try:
            # run rpitx -- record from audio, process and transmit
            command_line = "arecord -c1 -r48000 -D " +self.audiodev_in + \
            " -fS16_LE - | csdr convert_i16_f | csdr shift_addition_fc 0| csdr bandpass_fir_fft_cc 0.02 0.10 0.05 HAMMING | csdr realpart_cf | csdr rational_resampler_ff 1 2| csdr gain_ff 7000 | csdr convert_f_samplerf 20833 | sudo rpitx -i- -m RF -f %.3f" %(self.freq_tx)
            print(command_line)
           
            self.ptx = subprocess.Popen(command_line, shell = True)
            self.istx = True
        except:
            print("Something bad happend trying to transmit NFM")
     
    #kill the transmitter
    def stoptx(self):
        # kill all procecesses associated with transmitting.
        subprocess.run(shlex.split("sudo killall tune"))
        subprocess.run(shlex.split("sudo killall arecord"))
        subprocess.run(shlex.split("sudo killall rpitx"))
        self.istx = False
    
    
    #function receives Wideband broadcast FM using the rtlsdr and plays it on audiodev_out

    def WBFMrx(self):
        if self.isrx:
            print ("Receiver seems to be on already, try stopping first")
            return

        try:
            if self.audiodev_out == "plughw:CARD=Audio,DEV=0":
                command_line = "/usr/bin/rtl_fm -f %.3f -s 240k -g %f  -r 48000  -o 4  -A LUT  -E deemp - | aplay -t raw -r 48000 -f s16_LE -c 1 -D " \
                %(self.freq_rx*1000, self.sdr_gain)+ self.audiodev_out
        
            else:
                if self.audio_rcv_gain == 1.0:
                    command_line = "/usr/bin/rtl_fm -f %.3f -s 240k -g %f  -r 48000  -o 4  -A LUT  -E deemp - | aplay -t raw -r 48000 -f s16_LE -c 1 -D " \
                    %(self.freq_rx*1000, self.sdr_gain)+ self.audiodev_out

                else:
                    command_line = "/usr/bin/rtl_fm -f %.3f -s 240k -g %f  -r 48000  -o 4  -A LUT  -E deemp - | csdr convert_s16_f | csdr gain_ff %f | csdr convert_f_s16 | aplay -t raw -r 48000 -f s16_LE -c 1 -D " \
                    %(self.freq_rx*1000, self.sdr_gain, self.audio_rcv_gain)+ self.audiodev_out
                    
            print(command_line)

            self.prx = subprocess.Popen(command_line, shell = True)
            self.isrx = True
        except:
            print("Something bad happend trying to turn on the receiver")

    #function receives Narrow FM radio using rtl sdr and plays it on audiodev_in
    def NFMrx (self):
        if self.isrx:
            print ("Receiver seems to be on already, try stopping first")
            return
        
        try:
            if self.audiodev_out == "plughw:CARD=Audio,DEV=0":
                command_line = "/usr/bin/rtl_fm -f %.3f -s 48000 -g %d -l %d -p %d - | csdr mono2stereo_i16 | aplay -t raw -r 48000 -f s16_le  -D " \
                %(self.freq_rx*1000, self.sdr_gain, self.sdr_squelch, self.sdr_ppm)+ self.audiodev_out
        
            else:
                if self.audio_rcv_gain == 1.0:
                    command_line = "/usr/bin/rtl_fm -f %.3f -s 48000 -g %d -l %d -p %d - |   aplay -t raw -r 48000 -c 1 -f s16_le  -D " \
                    %(self.freq_rx*1000, self.sdr_gain, self.sdr_squelch, self.sdr_ppm)+ self.audiodev_out
                else:
                    command_line = "/usr/bin/rtl_fm -f %.3f -s 48000 -g %d -l %d -p %d - |  csdr convert_s16_f | csdr gain_ff %f | csdr convert_f_s16| aplay -t raw -r 48000 -c 1 -f s16_le  -D " \
                    %(self.freq_rx*1000, self.sdr_gain, self.sdr_squelch, self.sdr_ppm, self.audio_rcv_gain)+ self.audiodev_out               
            print(command_line)
        
            self.prx = subprocess.Popen(command_line, shell = True)
            self.isrx = True
        except:
            print("Something bad happend trying to turn on the receiver")
    
    #kill the receiver
    def stoprx(self):
        
        subprocess.run(shlex.split("killall rtl_fm")) 
        subprocess.run(shlex.split("killall aplay")) 
        self.isrx = False
        
    #kill both receiver and transmitter
    def close(self):
        self.stoprx();
        self.stoptx();
        

# function to compute average power spectrum
def avgPS( x, N=256, fs=1):
    M = (len(x)//N)
    x_ = np.reshape(x[:M*N],(M,N)) * np.hamming(N)[None,:]
    X = np.fft.fftshift(np.fft.fft(x_,axis=1),axes=1)
    return r_[-N/2.0:N/2.0]/N*fs, np.mean(abs(X)**2,axis=0)


# Plot an image of the spectrogram y, with the axis labeled with time tl,
# and frequency fl
#
# t_range -- time axis label, nt samples
# f_range -- frequency axis label, nf samples
# y -- spectrogram, nf by nt array
# dbf -- Dynamic range of the spect

def sg_plot( t_range, f_range, y, dbf = 60, fig = None) :
    eps = 10.0**(-dbf/20.0)  # minimum signal
    
    # find maximum
    y_max = abs(y).max()
    
    # compute 20*log magnitude, scaled to the max
    y_log = 20.0 * np.log10( (abs( y ) / y_max)*(1-eps) + eps )
    
    # rescale image intensity to 256
    img = 256*(y_log + dbf)/dbf - 1
    
    fig=plt.figure(figsize=(16,6))
    
    plt.imshow( np.flipud( 64.0*(y_log + dbf)/dbf ), extent= t_range  + f_range ,cmap=plt.cm.gray, aspect='auto')
    plt.xlabel('Time, s')
    plt.ylabel('Frequency, Hz')
    plt.tight_layout()
    
    return fig


def myspectrogram_hann_ovlp(x, m, fs, fc,dbf = 60):
    # Plot the spectrogram of x.
    # First take the original signal x and split it into blocks of length m
    # This corresponds to using a rectangular window %
    
    
    isreal_bool = np.isreal(x).all()
    
    # pad x up to a multiple of m 
    lx = len(x);
    nt = (lx + m - 1) // m
    x = np.append(x,zeros(-lx+nt*m))
    x = x.reshape((m//2,nt*2), order='F')
    x = np.concatenate((x,x),axis=0)
    x = x.reshape((m*nt*2,1),order='F')
    x = x[r_[m//2:len(x),np.ones(m//2)*(len(x)-1)].astype(int)].reshape((m,nt*2),order='F')
    
    
    xmw = x * np.hanning(m)[:,None];
    
    
    # frequency index
    t_range = [0.0, lx / fs]
    
    if isreal_bool:
        f_range = [ fc, fs / 2.0 + fc]
        xmf = np.fft.fft(xmw,len(xmw),axis=0)
        sg_plot(t_range, f_range, xmf[0:m//2,:],dbf=dbf)
        print (1)
    else:
        f_range = [-fs / 2.0 + fc, fs / 2.0 + fc]
        xmf = np.fft.fftshift( np.fft.fft( xmw ,len(xmw),axis=0), axes=0 )
        sg_plot(t_range, f_range, xmf,dbf = dbf)
    
    return t_range, f_range, xmf

