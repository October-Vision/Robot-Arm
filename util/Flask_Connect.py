import sys
sys.path.append("./")

import requests

from util.Audio_Func import AudioFunc
from util.Func import *


# Flask服务的URL和端口 
ip = 'http://192.168.28.84:5000'


#####################################################################################
#                                 多模态模型                                         #
#####################################################################################


# 初始化模型
def Initialize_Multi_Models(model_name, versions) -> str:
    url = ip + "/Initialize_Multi_Models"
    model = {"Model":model_name, "Versions":versions}
    # 发送POST请求
    response = requests.post(url, json=model)
    # 检查请求是否成功
    if response.status_code == 200:
        # 解析返回的JSON数据
        response_data = response.json()
        if response_data != 'Failed':
            print("初始化成功")
        else:
            print("初始化失败")
    else:
        print("请求失败，状态码为：", response.status_code)
    
    

# 调用多模态的聊天功能，返回音频
def Multi_Model_Chat_to_Chat(audio_file) -> str:
    url = ip + "/Multi_Model_Chat_to_Chat"
    with open(audio_file, 'rb') as audio:
        files = {'File': ('audio_file.wav', audio, 'audio/wav')}
        # 发送POST请求
        response = requests.post(url, files=files)

    # 检查请求是否成功
    if response.status_code == 200:
        # 解析返回的JSON数据
        response_data = response.json()
        if response_data != 'Failed':
            Decode_File(response_data, "Temp/result.wav")
            # 播放音频
            AudioFunc.Play_Audio("Temp/result.wav" )
        else:
            print(response_data)
    else:
        print("请求失败，状态码为：", response.status_code)
        
        
# 调用多模态后的聊天功能，返回的文字结果        
def Multi_Model_Chat_to_Result(audio_file) -> str:
    # Flask服务的URL和端口
    url = ip + "/Multi_Model_Chat_to_Result"
    
    with open(audio_file, 'rb') as audio:
        files = {'File': ('audio_file.wav', audio, 'audio/wav')}
        
        # 发送POST请求
        response = requests.post(url, files=files)

    # 检查请求是否成功
    if response.status_code == 200:
        # 解析返回的JSON数据
        response_data = response.json()
        if response_data != 'Failed':
            return response_data
        else:
            print(response_data)
    else:
        print("请求失败，状态码为：", response.status_code)
        
    return response_data       

# 调用多模态后的聊天功能，发送Message返回的文字结果  
def Multi_Model_Message_to_Result(message : list) -> str:
    # Flask服务的URL和端口
    url = ip + "/Multi_Model_Message_to_Result"

    data = {"Message" : message}
    # 发送POST请求
    response = requests.post(url, json=data)

    # 检查请求是否成功
    if response.status_code == 200:
        # 解析返回的JSON数据
        response_data = response.json()
        if response_data != 'Failed':
            return response_data
        else:
            print(response_data)
    else:
        print("请求失败，状态码为：", response.status_code)

    return response_data
        
        
# 调用多模态的检测功能，返回音频
def Multi_Model_Vision_to_Chat(audio_file, image_file) -> str:
    # Flask服务的URL和端口
    url = ip + "/Multi_Model_Vision_to_Chat"

    # 打开音频和图片文件并准备上传
    with open(audio_file, 'rb') as audio, \
        open(image_file, 'rb') as image:
        files = {
            'File': ('audio_file.wav', audio, 'audio/wav'),
            'Image': ('image_file.png', image, 'image/png')
        }
        # 发送POST请求
        response = requests.post(url, files=files)

    # 检查请求是否成功
    if response.status_code == 200:
        # 解析返回的JSON数据
        response_data = response.json()
        if response_data != 'Failed':
            Decode_File(response_data, "Temp/result.wav")
            # 播放音频
            AudioFunc.Play_Audio("Temp/result.wav")
        else:
            print(response_data)
            
    else:
        print("请求失败，状态码为：", response.status_code)
        
#####################################################################################
#                                    功能区                                         #
#####################################################################################

#发送音频进行翻译
def Multi_Model_Audio_to_Text(audio_file) -> str:
    # Flask服务的URL和端口
    url = ip + "/Multi_Model_Audio_to_Text"
    with open(audio_file, 'rb') as audio:
        files = {'File': ('audio_file.wav', audio, 'audio/wav')}
        # 发送POST请求
        response = requests.post(url, files=files)

    # 检查请求是否成功
    if response.status_code == 200:
        # 解析返回的JSON数据
        response_data = response.json()
        if response_data != 'Failed':
            return response_data
        else:
            print(response_data)
    else:
        print("请求失败，状态码为：", response.status_code)
        
    return response_data       
        
if __name__ == "__main__":
    # 初始化模型
    # Initialize_Multi_Models("Chat", "glm-4-9b-chat")
    
    message = [{"role": "user", "content": "你是谁"}]
    result =  Multi_Model_Message_to_Result(message)
    print(result)


