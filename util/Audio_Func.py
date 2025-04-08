import pyaudio
import numpy as np
import wave

from pydub import AudioSegment
from pydub.playback import play

class AudioFunc:
    def __init__(self):
        self.stop = False #停止录音
    
    def Transcribe_Audio(self, save_file, mindb):
        CHUNK = 1024
        FORMAT = pyaudio.paInt16
        CHANNELS = 1
        RATE = 44100
        WAVE_OUTPUT_FILENAME = save_file  # 保存的文件名

        p = pyaudio.PyAudio()  # 初始化

        stream = p.open(format=FORMAT,
                        channels=CHANNELS,
                        rate=RATE,
                        input=True,
                        frames_per_buffer=CHUNK)  # 创建录音文件
        frames = []
        print("Start")
        # 如果mindb 不是布尔类型
        if not isinstance(mindb, bool):
            while True:
                data = stream.read(CHUNK, exception_on_overflow=False)
                audio_data = np.frombuffer(data, dtype=np.short)  # 将字节数据转换为 NumPy 数组
                # print(audio_data.shape)
                temp = np.max(audio_data)  # 返回数组中的最大值，即音频数据的最大振幅
                print(temp)
                # 最小声音，大于则开始录音，否则结束
                if temp >= mindb:
                    detect_count = 0
                    while True:
                        data = stream.read(CHUNK, exception_on_overflow=False)
                        audio_data = np.frombuffer(data, dtype=np.short)  # 将字节数据转换为 NumPy 数组
                        temp = np.max(audio_data)  # 返回数组中的最大值，即音频数据的最大振幅
                        print(temp)
                        frames.append(data)
                        if temp < mindb:
                            detect_count += 1
                            if detect_count == 40:
                                break
                        else:
                            detect_count = 0
                    break
        else:
            while True:
                data = stream.read(CHUNK, exception_on_overflow=False)
                frames.append(data)
                if self.stop:
                    self.stop = False
                    break
        print("OFF")
        wf = wave.open(WAVE_OUTPUT_FILENAME, 'wb')  # 保存
        wf.setnchannels(CHANNELS)
        wf.setsampwidth(p.get_sample_size(FORMAT))
        wf.setframerate(RATE)
        wf.writeframes(b''.join(frames))
        wf.close()
        
        return True
        
    #播放音频
    def Play_Audio(self, audio_file):
        audio = AudioSegment.from_wav(audio_file)
        # 播放音频
        play(audio)
        