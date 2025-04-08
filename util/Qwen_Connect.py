import json
import dashscope
from dashscope.audio.asr import Recognition
from openai import OpenAI
from http import HTTPStatus
import dashscope
import requests


with open('config/argument.json', 'r', encoding='utf-8') as f:
        argument = json.load(f)
key = argument["key"]


def Qwen_Chagt_to_Result(message : list) -> str:
    try:
        client = OpenAI(
            # 若没有配置环境变量，请用百炼API Key将下行替换为：api_key="sk-xxx",
            api_key = key,
            base_url="https://dashscope.aliyuncs.com/compatible-mode/v1",
        )

        completion = client.chat.completions.create(
            model="qwen-plus",  # 模型列表：https://help.aliyun.com/zh/model-studio/getting-started/models
            messages= message
        )
        return completion.choices[0].message.content
    except Exception as e:
        print(f"错误信息：{e}")
        print("请参考文档：https://help.aliyun.com/zh/model-studio/developer-reference/error-code")
        return None
    
def Qwen_Audio_to_Text(audio_file) -> str:
    dashscope.api_key = key 

    recognition = Recognition(model='paraformer-realtime-v2',
                            format='wav',
                            sample_rate=44100,
                            callback=None)
    result = recognition.call(audio_file)
    if result.status_code == HTTPStatus.OK:
        results = result.get_sentence()
        return results[0]["text"]
    else:
        print('Error: ', result.message)
            
if __name__ == "__main__":
    text = Qwen_Chagt_to_Result([{"role": "user", "content": "你是谁"}])
    print(text)
    