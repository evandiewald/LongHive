import librosa
import matplotlib.pyplot as plt
import librosa.display
import numpy as np
from os import listdir, rename
from scipy import signal

n = 100000
audio_path_list = ['Processed Data/No Queen', 'Processed Data/Queen', 'Processed Data/NoBee']
for iPath in range(len(audio_path_list)):
    onlyfiles = listdir(audio_path_list[iPath])
    for iFile in range(len(onlyfiles)):
        fname = audio_path_list[iPath] + '/' + onlyfiles[iFile]
        try:
            y, sr = librosa.load(fname)
            # filt = signal.butter(4, 100, 'high')
            # y = signal.sosfilt(filt, y)
        except ValueError:
            continue
        snippets = [y[i:i + n] for i in range(0, len(y), n)]
        for i in range(len(snippets)):
            if len(snippets[i]) < n:
                continue
            else:
                mfccs = librosa.feature.melspectrogram(y=snippets[i], sr=sr, fmax=sr/2)
                # mfccs = np.mean(mfccs.T, axis=0)
                mfccs = librosa.power_to_db(mfccs, ref=np.max)

                fig = plt.figure(figsize=(5, 4))
                librosa.display.specshow(mfccs, x_axis='time')
                plt.clim(-60, 0)
                plt.tight_layout()
                # plt.show()
                if iPath == 0:
                    destpath = 'Melspectrograms2/No Queen/'
                elif iPath == 1:
                    destpath = 'Melspectrograms2/Queen/'
                elif iPath == 2:
                    destpath = 'Melspectrograms2/NoBee/'
                dest = destpath + str(len(listdir(destpath))).zfill(5) + '.png'
                plt.gcf()
                plt.savefig(dest)
                plt.close()

